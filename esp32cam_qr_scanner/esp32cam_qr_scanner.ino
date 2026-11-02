/*
 * ESP32-CAM  —  QR Command Scanner (Wi-Fi)
 * ----------------------------------------
 * Scans QR codes and sends the command to the motor ESP32 over the same
 * Wi-Fi network (UDP). The motor ESP32 answers every command with an ACK;
 * commands that are not acknowledged are retried.
 *
 * Valid QR contents (case does not matter):
 *   FORWARD   BACKWARD   LEFT   RIGHT   STOP
 *   (or the short forms F  B  L  R  S)
 *
 * Board:    "AI Thinker ESP32-CAM"
 * Library:  ESP32QRCodeReader (only its quirc QR decoder is used here)
 *
 * Works with OV2640 and GC2145 camera sensors: frames are captured as
 * RGB565 and converted to grayscale in software.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include <ESP32QRCodeReader.h>   // brings in the quirc decoder
#include "quirc/quirc.h"

// ---------- settings ----------
const char *WIFI_SSID = "YOUR_WIFI_NAME";      // same network on both boards (2.4 GHz)
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
const uint16_t UDP_PORT = 4210;   // must match the motor ESP32

#define FLASH_LED_PIN 4           // on-board white flash LED
#define REPEAT_MS     700         // resend a QR that stays in view (keeps the car moving)
#define ACK_TIMEOUT   200         // ms to wait for the motor ESP32 to answer
#define SEND_RETRIES  3

// ---------- AI Thinker ESP32-CAM pins ----------
#define CAM_PWDN   32
#define CAM_RESET  -1
#define CAM_XCLK    0
#define CAM_SIOD   26
#define CAM_SIOC   27
#define CAM_Y9     35
#define CAM_Y8     34
#define CAM_Y7     39
#define CAM_Y6     36
#define CAM_Y5     21
#define CAM_Y4     19
#define CAM_Y3     18
#define CAM_Y2      5
#define CAM_VSYNC  25
#define CAM_HREF   23
#define CAM_PCLK   22

WiFiUDP udp;
IPAddress motorIP;
bool motorKnown = false;
uint32_t seq = 0;

struct quirc *qr = nullptr;
struct quirc_code qrCode;     // large structs: keep them off the stack
struct quirc_data qrData;

String lastCmd = "";
unsigned long lastSentAt = 0;

// statistics, printed every 5 s
uint32_t statFrames = 0, statFound = 0, statDecoded = 0;
uint32_t statBrightness = 0;
unsigned long statStart = 0;

// ================= camera =================

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2;  c.pin_d1 = CAM_Y3;  c.pin_d2 = CAM_Y4;  c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6;  c.pin_d5 = CAM_Y7;  c.pin_d6 = CAM_Y8;  c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK;
  c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC;
  c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD;
  c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = CAM_PWDN;
  c.pin_reset = CAM_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;   // supported by every sensor, incl. GC2145
  c.frame_size = FRAMESIZE_QVGA;       // 320x240
  c.jpeg_quality = 12;
  if (psramFound()) {
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.fb_count = 1;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.fb_count = 1;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Camera sensor PID: 0x%x\n", s->id.PID);
  return true;
}

// Convert a camera frame to 8-bit grayscale. Returns the average brightness.
uint8_t frameToGray(const camera_fb_t *fb, uint8_t *dst, size_t pixels) {
  uint32_t sum = 0;
  if (fb->format == PIXFORMAT_RGB565) {
    const uint8_t *src = fb->buf;
    for (size_t i = 0; i < pixels; i++, src += 2) {
      uint8_t r = src[0] & 0xF8;
      uint8_t g = ((src[0] & 0x07) << 5) | ((src[1] & 0xE0) >> 3);
      uint8_t b = (src[1] & 0x1F) << 3;
      uint8_t y = (r * 77 + g * 150 + b * 29) >> 8;
      dst[i] = y;
      sum += y;
    }
  } else {  // already grayscale
    memcpy(dst, fb->buf, pixels);
    for (size_t i = 0; i < pixels; i += 16) sum += dst[i] * 16;
  }
  return sum / pixels;
}

// Mirror a QR grid (some camera modules deliver mirrored images).
void flipCode(struct quirc_code *code) {
  static struct quirc_code flipped;
  memset(flipped.cell_bitmap, 0, sizeof(flipped.cell_bitmap));
  int n = code->size;
  for (int y = 0; y < n; y++) {
    for (int x = 0; x < n; x++) {
      int src = x * n + y;   // transpose
      int dst = y * n + x;
      if (code->cell_bitmap[src >> 3] & (1 << (src & 7))) {
        flipped.cell_bitmap[dst >> 3] |= (1 << (dst & 7));
      }
    }
  }
  memcpy(code->cell_bitmap, flipped.cell_bitmap, sizeof(flipped.cell_bitmap));
}

// ================= network =================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // lower latency
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Lower TX power = smaller current peaks (camera + radio can brown out the
  // ESP32-CAM on weak USB/5 V supplies). Plenty of range for a nearby hotspot.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  Serial.printf("Connecting to Wi-Fi \"%s\"", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Wi-Fi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("Wi-Fi not connected yet (will keep retrying in the background)");
  }
}

// Send a command and wait for "ACK <seq>". The first ACK tells us the motor
// ESP32's IP; until then (or after it stops answering) we use broadcast.
bool sendCommand(const String &cmd) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  not sent: Wi-Fi disconnected");
    return false;
  }

  seq++;
  String msg = "CMD " + String(seq) + " " + cmd;
  String expected = "ACK " + String(seq);

  for (int attempt = 1; attempt <= SEND_RETRIES; attempt++) {
    IPAddress dest = motorKnown ? motorIP : WiFi.broadcastIP();
    udp.beginPacket(dest, UDP_PORT);
    udp.print(msg);
    udp.endPacket();

    unsigned long t = millis();
    while (millis() - t < ACK_TIMEOUT) {
      int len = udp.parsePacket();
      if (len > 0) {
        char buf[32];
        int n = udp.read(buf, sizeof(buf) - 1);
        buf[n > 0 ? n : 0] = '\0';
        if (expected == buf) {
          if (!motorKnown || motorIP != udp.remoteIP()) {
            motorIP = udp.remoteIP();
            motorKnown = true;
            Serial.printf("  motor ESP32 found at %s\n", motorIP.toString().c_str());
          }
          return true;
        }
      }
      delay(2);
    }
  }

  motorKnown = false;   // rediscover by broadcast next time
  return false;
}

// ================= commands =================

// Turn whatever text is in the QR into one of our commands, or "" if invalid.
String normalizeCommand(String text) {
  text.trim();
  text.toUpperCase();

  if (text == "FORWARD"  || text == "F") return "FORWARD";
  if (text == "BACKWARD" || text == "B" || text == "BACK") return "BACKWARD";
  if (text == "LEFT"     || text == "L") return "LEFT";
  if (text == "RIGHT"    || text == "R") return "RIGHT";
  if (text == "STOP"     || text == "S") return "STOP";
  return "";
}

void blinkFlash() {
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(30);
  digitalWrite(FLASH_LED_PIN, LOW);
}

void handleQrText(const char *text) {
  String cmd = normalizeCommand(text);
  if (cmd == "") {
    Serial.printf("QR read, but not a command: \"%s\"\n", text);
    return;
  }

  unsigned long now = millis();
  // A QR held in view is decoded several times per second; send it again
  // only every REPEAT_MS (or immediately if the command changed).
  if (cmd == lastCmd && now - lastSentAt < REPEAT_MS) return;
  lastCmd = cmd;
  lastSentAt = now;

  blinkFlash();
  bool ok = sendCommand(cmd);
  Serial.printf("QR: %-8s -> %s\n", cmd.c_str(), ok ? "delivered (ACK)" : "NOT delivered (no ACK)");
}

// ================= main =================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32-CAM QR scanner ===");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.printf("PSRAM: %s\n", psramFound() ? "found" : "NOT found");

  // Camera first, so its frame buffers get memory before Wi-Fi does.
  if (!initCamera()) {
    Serial.println("ERROR: camera init failed (check ribbon cable / power). Restarting in 5 s");
    delay(5000);
    ESP.restart();
  }

  qr = quirc_new();
  if (!qr || quirc_resize(qr, 320, 240) < 0) {
    Serial.println("ERROR: not enough memory for the QR decoder. Restarting in 5 s");
    delay(5000);
    ESP.restart();
  }

  connectWiFi();
  udp.begin(UDP_PORT);

  Serial.println("Ready. Show a QR code to the camera.");
  statStart = millis();
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(100);
    return;
  }

  int w, h;
  uint8_t *image = quirc_begin(qr, &w, &h);
  if (fb->width != w || fb->height != h) {
    // frame size changed (should not happen) - resize decoder and skip frame
    esp_camera_fb_return(fb);
    quirc_resize(qr, fb->width, fb->height);
    return;
  }
  uint8_t brightness = frameToGray(fb, image, (size_t)w * h);
  if (Serial.available() && Serial.read() == 'd') {
    // Debug: send 'd' in the Serial Monitor to dump this frame as hex.
    Serial.printf("RAW fmt=%d len=%u first=%02x %02x %02x %02x\n", fb->format, fb->len,
                  fb->buf[0], fb->buf[1], fb->buf[2], fb->buf[3]);
    Serial.printf("FRAME %d %d\n", w, h);
    for (int i = 0; i < w * h; i++) {
      Serial.printf("%02x", image[i]);
      if (i % w == w - 1) Serial.println();
    }
    Serial.println("END");
  }
  esp_camera_fb_return(fb);
  quirc_end(qr);

  statFrames++;
  statBrightness += brightness;

  int count = quirc_count(qr);
  if (count > 0) statFound++;

  for (int i = 0; i < count; i++) {
    quirc_extract(qr, i, &qrCode);
    quirc_decode_error_t err = quirc_decode(&qrCode, &qrData);
    if (err == QUIRC_ERROR_DATA_ECC || err == QUIRC_ERROR_FORMAT_ECC) {
      flipCode(&qrCode);                     // maybe the image is mirrored
      err = quirc_decode(&qrCode, &qrData);
    }
    if (err == QUIRC_SUCCESS && qrData.payload_len < QUIRC_MAX_PAYLOAD) {
      statDecoded++;
      qrData.payload[qrData.payload_len] = '\0';
      handleQrText((const char *)qrData.payload);
    }
  }

  // Health report every 5 s: helps to see if the camera sees the QR at all.
  if (millis() - statStart >= 5000) {
    Serial.printf("[status] %u frames, QR seen in %u, decoded %u, brightness %u/255, Wi-Fi %s\n",
                  statFrames, statFound, statDecoded,
                  statFrames ? statBrightness / statFrames : 0,
                  WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
    statFrames = statFound = statDecoded = statBrightness = 0;
    statStart = millis();
  }
}
