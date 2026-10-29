# RC Tank – PS3 Controlled, CL 9030 Brushed ESC

An ESP32-based RC tank driven by two CL 9030 brushed ESCs (one per track, each fed by a single-channel PWM signal). The tank is driven with a Bluetooth PS3 (DualShock 3) controller. A WiFi access point provides a configuration page only; it cannot drive the tank.

- **Target:** ESP32 Dev Module (30-pin WROOM, Classic Bluetooth for PS3)
- **IDE:** Arduino IDE (ESP32 core 3.x)
- **ESCs:** 2× CL 9030 brushed ESC, 3-wire receiver lead
- **Driving:** PS3 DualShock 3 only
- **Config:** WiFi AP web page and serial console

---

## Table of Contents

1. [Arduino IDE Settings](#arduino-ide-settings)
2. [ESC Wiring](#esc-wiring)
3. [Driving with the PS3 Controller](#driving-with-the-ps3-controller)
4. [WiFi Configuration Page](#wifi-configuration-page)
5. [HTTP Endpoints](#http-endpoints)
6. [Serial Console](#serial-console)

---

## Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module (WROOM, 30-pin classic) |
| Flash Size | 4MB |
| Partition Scheme | Minimal SPIFFS (1.9MB APP) |

---

## ESC Wiring

Each CL 9030 ESC has a 3-wire receiver lead.

| Wire | Connect to | Notes |
|---|---|---|
| White | GPIO signal | Left ESC → **GPIO 5**, Right ESC → **GPIO 4** |
| Black | GND | **Required** |
| Red | 5V BEC | Optional. Leave unconnected if the ESP32 is powered by USB |

### Startup

After upload (or reset), the Serial Monitor shows a **2.5 s neutral hold**. Connect the ESC battery during this window so the ESCs arm at neutral.

---

## Driving with the PS3 Controller

Driving is done **only** with a PS3 (DualShock 3) controller.

| Input | Function |
|---|---|
| Left stick (up/down) | Left track |
| Right stick (up/down) | Right track |
| PS button | All stop |
| Controller disconnect | All stop |

Both sticks up drives straight ahead. Opposite directions make a tank turn.

### Pairing

1. Open the Serial Monitor at 115200 baud and type `btmac` to get the ESP32 Bluetooth MAC address.
2. Plug the PS3 controller into a PC/Mac via USB.
3. Write that MAC address into the controller with SixaxisPairTool / sixaxispairer.
4. Unplug USB and press the PS button to connect.

### Library

Uses a patched PS3 Controller Host library for ESP32 core 3.x, included at `esp/libraries/PS3_Controller_Host/`.

---

## WiFi Configuration Page

The ESP32 runs a WiFi access point for **configuration only**. You cannot drive the tank over WiFi.

| Item | Value |
|---|---|
| SSID | `TankRC` |
| Password | `tank1234` |
| IP address | `192.168.4.1` |

Connect to the `TankRC` network and open <http://192.168.4.1>.

---

## HTTP Endpoints

| Method | Path | Parameters | Description |
|---|---|---|---|
| GET | `/` | none | Config page |
| GET | `/status` | none | JSON state |
| GET | `/config` | `top`=19.0..100 | Set the maximum ESC throttle % at full stick (saved to flash) |
| GET | `/allstop` | none | Stop both tracks |

Example:

```
http://192.168.4.1/config?top=60
http://192.168.4.1/allstop
```

---

## Serial Console

Open the Serial Monitor at **115200** baud.

| Command | Description |
|---|---|
| `l <pct>` | Set left track throttle % |
| `r <pct>` | Set right track throttle % |
| `top <pct>` | Set max ESC throttle % at full stick |
| `stop` | All stop |
| `status` | Print current state |
| `btmac` | Print the Bluetooth MAC (for PS3 pairing) |