/*
 * ESP32  —  4-Wheel Motor Controller (L298N, Wi-Fi)
 * -------------------------------------------------
 * Joins the same Wi-Fi network as the ESP32-CAM, receives commands over UDP
 * (and answers each one with an ACK), and drives 4 DC motors through one L298N:
 *   - Left  front + left  rear motors in parallel on OUT1 / OUT2  (Motor A)
 *   - Right front + right rear motors in parallel on OUT3 / OUT4  (Motor B)
 *
 * Commands: FORWARD  BACKWARD  LEFT  RIGHT  STOP
 * Other ways to send them (for testing):
 *   - Serial Monitor: type the command and press Enter
 *   - Browser: open http://<ip shown at startup>/  and press the buttons
 *
 * Board: "ESP32 Dev Module"
 *
 * Wiring:
 *   ESP32 GND  ---  L298N GND  ---  Battery (-)
 *
 *   L298N ENA -> GPIO25   (remove the ENA jumper)
 *   L298N IN1 -> GPIO26
 *   L298N IN2 -> GPIO27
 *   L298N IN3 -> GPIO32
 *   L298N IN4 -> GPIO33
 *   L298N ENB -> GPIO14   (remove the ENB jumper)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// ---------- settings ----------
const char *WIFI_SSID = "YOUR_WIFI_NAME";      // same network on both boards (2.4 GHz)
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";
const uint16_t UDP_PORT = 4210;   // must match the ESP32-CAM

// ---- L298N pins ----
#define ENA 25   // left side speed (PWM)
#define IN1 26
#define IN2 27
#define IN3 32
#define IN4 33
#define ENB 14   // right side speed (PWM)

#define STATUS_LED 2   // on-board LED: on = Wi-Fi connected

// ---- tuning ----
#define DRIVE_SPEED 200    // 0..255
#define TURN_SPEED  180    // 0..255
#define MOVE_TIME   1000   // ms each command runs before auto-stop (0 = run until STOP)

WiFiUDP udp;
WebServer server(80);

unsigned long moveStartedAt = 0;
bool moving = false;
bool wifiWasConnected = false;
uint32_t lastSeq = 0;

// ================= motors =================

// dir: +1 forward, -1 backward, 0 stop
void setSide(int pinA, int pinB, int enPin, int dir, int speed) {
  digitalWrite(pinA, dir > 0 ? HIGH : LOW);
  digitalWrite(pinB, dir < 0 ? HIGH : LOW);
  analogWrite(enPin, dir == 0 ? 0 : speed);
}

void drive(int leftDir, int rightDir, int speed) {
  setSide(IN1, IN2, ENA, leftDir,  speed);
  setSide(IN3, IN4, ENB, rightDir, speed);
}

void stopMotors() {
  drive(0, 0, 0);
  moving = false;
}

void forward()  { drive(+1, +1, DRIVE_SPEED); }
void backward() { drive(-1, -1, DRIVE_SPEED); }
void left()     { drive(-1, +1, TURN_SPEED); }   // spin in place to the left
void right()    { drive(+1, -1, TURN_SPEED); }   // spin in place to the right

// Returns false for an unknown command.
bool handleCommand(String cmd, const char *source) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return false;

  Serial.printf("Command: %-8s (from %s)\n", cmd.c_str(), source);

  if      (cmd == "FORWARD")  forward();
  else if (cmd == "BACKWARD") backward();
  else if (cmd == "LEFT")     left();
  else if (cmd == "RIGHT")    right();
  else if (cmd == "STOP")   { stopMotors(); return true; }
  else { Serial.println("  (ignored: unknown command)"); return false; }

  moving = true;
  moveStartedAt = millis();
  return true;
}

// ================= network =================

// Packet format from the ESP32-CAM: "CMD <seq> <COMMAND>"  ->  reply "ACK <seq>"
void handleUdp() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[64];
  int n = udp.read(buf, sizeof(buf) - 1);
  buf[n > 0 ? n : 0] = '\0';

  uint32_t seq;
  char cmd[16];
  if (sscanf(buf, "CMD %lu %15s", &seq, cmd) != 2) return;   // not for us

  // Always ACK (the camera may have missed our previous ACK) ...
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.printf("ACK %lu", seq);
  udp.endPacket();

  // ... but run a retried packet only once.
  if (seq == lastSeq) return;
  lastSeq = seq;
  handleCommand(cmd, udp.remoteIP().toString().c_str());
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Motor car</title><style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee}
.g{display:grid;grid-template-columns:repeat(3,90px);gap:10px;justify-content:center;margin-top:30px}
button{height:90px;font-size:16px;border:0;border-radius:12px;background:#2a6;color:#fff}
#s{background:#c33}
</style></head><body><h2>Motor car</h2><div class="g">
<span></span><button onclick="c('FORWARD')">FORWARD</button><span></span>
<button onclick="c('LEFT')">LEFT</button><button id="s" onclick="c('STOP')">STOP</button><button onclick="c('RIGHT')">RIGHT</button>
<span></span><button onclick="c('BACKWARD')">BACKWARD</button><span></span>
</div><p id="r"></p><script>
function c(x){fetch('/cmd?c='+x).then(r=>r.text()).then(t=>document.getElementById('r').textContent=t)}
</script></body></html>)HTML";

void setupWebServer() {
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/cmd", []() {
    bool ok = handleCommand(server.arg("c"), "web");
    server.send(ok ? 200 : 400, "text/plain", ok ? "OK " + server.arg("c") : "unknown command");
  });
  server.begin();
}

void checkWiFi() {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wifiWasConnected) {
    Serial.printf("Wi-Fi connected, IP: %s   (control page: http://%s/)\n",
                  WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
  } else if (!connected && wifiWasConnected) {
    Serial.println("Wi-Fi lost -> motors stopped, reconnecting...");
    stopMotors();   // safety: never keep driving without a link
  }
  digitalWrite(STATUS_LED, connected ? HIGH : LOW);
  wifiWasConnected = connected;
}

// ================= main =================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Motor controller ===");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  stopMotors();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to Wi-Fi \"%s\"", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected yet (will keep retrying in the background)");
  }

  udp.begin(UDP_PORT);
  setupWebServer();
  checkWiFi();

  Serial.println("Ready. Waiting for commands (QR camera, browser, or Serial Monitor).");
}

void loop() {
  checkWiFi();
  handleUdp();
  server.handleClient();

  if (Serial.available()) {          // manual testing from Serial Monitor
    handleCommand(Serial.readStringUntil('\n'), "serial");
  }

  // Safety: stop after MOVE_TIME so the car doesn't run away.
  if (moving && MOVE_TIME > 0 && millis() - moveStartedAt >= MOVE_TIME) {
    stopMotors();
    Serial.println("Auto-stop");
  }
}
