# QR-Controlled 4-Wheel Car (ESP32-CAM + ESP32 + L298N, Wi-Fi)

Show a QR code to the ESP32-CAM → it decodes the command → sends it over Wi-Fi (UDP) to the motor ESP32 → the motor ESP32 confirms (ACK) and drives the 4 wheels through the L298N.

```
 [QR code] --> ESP32-CAM ~~(Wi-Fi, UDP port 4210, with ACK)~~> ESP32 --> L298N --> 4 motors
```

Both boards join the same Wi-Fi network: put its name and password in `WIFI_SSID` / `WIFI_PASS` at the top of **both** sketches. The camera finds the motor ESP32 automatically (broadcast, then direct), so no IP addresses need to be typed in.

## Repository layout
```
esp32cam_qr_scanner/       ESP32-CAM sketch: scan QR -> send command over Wi-Fi (UDP)
esp32_motor_controller/    ESP32 sketch: receive command -> drive L298N (+ web control page)
libraries/ESP32QRCodeReader/  patched QR library (copy into Documents\Arduino\libraries)
qr_codes/                  printable QR codes (PNG + A4 PDF)
```

## Parts
- ESP32-CAM (AI Thinker; OV2640 or GC2145 sensor) + USB programmer (ESP32-CAM-MB or FTDI)
- ESP32 Dev board
- L298N motor driver
- 4 × DC gear motors + wheels + chassis
- Battery: 2 × 18650 (7.4 V) or a 7–12 V pack
- 5 V buck converter to power both ESP boards
- A **2.4 GHz** Wi-Fi network (router, phone hotspot, or Windows Mobile Hotspot)

## Wiring

### ESP32 → L298N
| ESP32 | L298N |
|-------|-------|
| GPIO25 | ENA (take off the jumper) |
| GPIO26 | IN1 |
| GPIO27 | IN2 |
| GPIO32 | IN3 |
| GPIO33 | IN4 |
| GPIO14 | ENB (take off the jumper) |
| GND | GND |

### Motors → L298N
| L298N output | Motors (wired in parallel) |
|--------------|---------------------------|
| OUT1 / OUT2 | Left front + Left rear |
| OUT3 / OUT4 | Right front + Right rear |

### Power
| From | To |
|------|----|
| Battery + | L298N 12V terminal and buck converter IN+ |
| Battery − | L298N GND and buck converter IN− |
| Buck 5V out | ESP32 VIN and ESP32-CAM 5V |
| Buck GND | ESP32 GND and ESP32-CAM GND |

The ESP32 GND **must** be connected to the L298N GND and battery (−). The ESP32-CAM has no wires to the other boards except power.

The ESP32-CAM needs a steady 5 V supply (≥ 500 mA). If it keeps resetting ("brownout"), the supply is too weak.

## Software setup (Arduino IDE)
1. Boards Manager: install **esp32 by Espressif Systems** (tested with 3.3.12).
2. Copy the folder `libraries/ESP32QRCodeReader` from this repo into `Documents\Arduino\libraries` and restart the IDE. This is a patched copy of [alvarowolfx/ESP32QRCodeReader](https://github.com/alvarowolfx/ESP32QRCodeReader) (MIT): headers moved to `src/`, missing declarations added for ESP32 core 3.x, and PSRAM made optional. The original from the Library Manager / GitHub will **not** compile. The scanner uses only its quirc decoder.
3. Upload `esp32cam_qr_scanner/esp32cam_qr_scanner.ino` — Board **AI Thinker ESP32-CAM**.
4. Upload `esp32_motor_controller/esp32_motor_controller.ino` — Board **ESP32 Dev Module**.

## QR codes
Printable codes are in `qr_codes/` (`ALL_QR_A4.pdf` = all five on one page). They contain plain text: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`.
Hold one 15–30 cm from the camera with the whole code (plus a white border) in view.

## What you see in the Serial Monitors (115200 baud)

ESP32-CAM:
```
Camera sensor PID: 0x2145
Wi-Fi connected, IP: 192.168.137.145
Ready. Show a QR code to the camera.
[status] 36 frames, QR seen in 0, decoded 0, brightness 164/255, Wi-Fi OK
  motor ESP32 found at 192.168.137.xx
QR: FORWARD  -> delivered (ACK)
```
- `brightness` near 0 → lens covered / no light. Near 255 → overexposed.
- `QR seen` > 0 but `decoded` 0 → QR blurry, too small, or cut off.
- `NOT delivered (no ACK)` → motor ESP32 not on the network / not running.
- Send `d` in the Serial Monitor to dump one grayscale frame as hex (debugging).

Motor ESP32:
```
Wi-Fi connected, IP: 192.168.137.xx   (control page: http://192.168.137.xx/)
Command: FORWARD  (from 192.168.137.145)
Auto-stop
```

## Testing step by step
1. **Motors:** type `FORWARD`, `LEFT`, … in the motor ESP32 Serial Monitor, or open the control page in a browser on the same network and press the buttons. If a side spins the wrong way, swap that side's two motor wires.
2. **Camera:** show a QR — the camera prints `QR: FORWARD -> delivered (ACK)`, the motor ESP32 prints `Command: FORWARD`.

## Behaviour and tuning
- Each command runs for `MOVE_TIME` (1 s) and then stops. The camera re-sends a QR that stays in view every `REPEAT_MS` (0.7 s), so holding a QR keeps the car moving; taking it away stops the car.
- `MOVE_TIME 0` → keep moving until a `STOP` QR is shown.
- If Wi-Fi drops, the motor ESP32 stops the motors immediately.
- `DRIVE_SPEED` / `TURN_SPEED` (0–255) set the speeds.
- LEFT / RIGHT spin the car in place.

## Notes on the GC2145 camera sensor
Many recent ESP32-CAM boards ship with a GC2145 instead of an OV2640. It has no JPEG or usable grayscale mode, and with two frame buffers it returns black frames. The scanner therefore captures RGB565 with a single frame buffer and converts to grayscale in software. This also works with the OV2640.
