# Setup Guide — ESP32-CAM Dual-Axis CCTV Surveillance System

## Table of Contents

1. [Hardware Required](#hardware-required)
2. [Wiring](#wiring)
3. [Software Setup](#software-setup)
4. [Configuration](#configuration)
5. [Flashing](#flashing)
6. [Usage](#usage)
7. [3D Printing](#3d-printing)
8. [PCB](#pcb)
9. [Troubleshooting](#troubleshooting)

\---

## Hardware Required

|Component|Qty|Notes|
|-|-|-|
|AI Thinker ESP32-CAM (OV3660)|1|Main controller + camera|
|MG90S Metal Gear Servo|2|Pan + Tilt|
|Pan-Tilt Bracket|1|Standard SG90/MG90S compatible|
|HC-SR501 PIR Sensor|1|Optional — for hardware motion detect|
|850/940nm IR LED 5mm|4|Night vision|
|LDR (Light Dependent Resistor)|1|Auto night vision switching|
|2N2222A NPN Transistor (TO-92)|1|IR LED switching|
|330Ω Resistor 1/4W|1|Transistor base resistor|
|100Ω Resistor 1/4W|4|IR LED current limiters|
|10kΩ Resistor 1/4W|1|LDR voltage divider|
|LM2596S Buck Converter|1|Set output to 5V|
|18650 Battery Holder (2 cell)|1|With ON/OFF switch|
|18650 Battery 3.7V|2|Recommended: 2600mAh+|
|FTDI FT232RL / ESP32-CAM-MB|1|For flashing only|
|Jumper Wires|—|Male-to-female|
|Zero PCB / Perfboard|1|For IR LED circuit|

\---

## Wiring

### Power

```
18650 Pack (+) → LM2596S IN+
18650 Pack (−) → LM2596S IN−
LM2596S OUT+    → ESP32-CAM 5V
LM2596S OUT+    → Pan servo VCC 
LM2596S OUT+    → Tilt servo VCC 
LM2596S OUT−    → ESP32-CAM GND
LM2596S OUT−    → Pan servo GND 
LM2596S OUT−    → Tilt servo GND 
```

> ⚠️ Set LM2596S output to exactly 5.0V before connecting anything

### Servos

```
Pan  servo signal  → GPIO 14
Tilt servo signal  → GPIO 15
```

### IR LED Circuit (LDR auto-switching)

```
5V ──\[10kΩ]──┬──\[330Ω]──→ 2N2222 BASE (middle leg)
             │
           \[LDR]
             │
            GND

2N2222 EMITTER (left leg)    → GND
2N2222 COLLECTOR (right leg) → 4× \[100Ω] → IR LED (+)
                                             IR LED (−) → GND
```

### 2N2222 Pinout (flat face toward you, legs down)

```
Left = Emitter | Middle = Base | Right = Collector
```

### GPIO Summary

```
GPIO 4  → IR LED / Onboard flash LED
GPIO 14 → Pan servo signal
GPIO 15 → Tilt servo signal
```

\---

## Software Setup

### 1\. Install Arduino IDE

Download from [arduino.cc](https://www.arduino.cc/en/software)

### 2\. Add ESP32 Board Support

* Open Arduino IDE
* Go to **File → Preferences**
* Add to Additional Board Manager URLs:

```
https://dl.espressif.com/dl/package\_esp32\_index.json
```

* Go to **Tools → Board → Boards Manager**
* Search `esp32` → Install **esp32 by Espressif Systems**

### 3\. Install Libraries

Go to **Sketch → Include Library → Manage Libraries**

|Library|Author|
|-|-|
|ESP32Servo|Kevin Harrington|

### 4\. Select Board

**Tools → Board → AI Thinker ESP32-CAM**

\---

## Configuration

Open `firmware/ESP32\_CAM\_Surveillance.ino` and edit these lines at the top:

```cpp
// WiFi credentials
const char\* WIFI\_SSID = "YOUR\_WIFI\_NAME";
const char\* WIFI\_PASS = "YOUR\_WIFI\_PASSWORD";

// Static IP — change to match your network
// Find your gateway: Windows CMD → ipconfig → Default Gateway
IPAddress STATIC\_IP(192, 168, 1, 100);  // ← pick any free IP
IPAddress GATEWAY  (192, 168, 1,   1);  // ← your router IP
IPAddress SUBNET   (255, 255, 255,  0);  // ← leave this
IPAddress DNS      (  8,   8,   8,  8);  // ← leave this
```

> Example: if your gateway is 192.168.108.1 → use 192.168.108.150 for STATIC\_IP

\---

## Flashing

### Using FTDI Module

```
FTDI GND → ESP32-CAM GND
FTDI 5V  → ESP32-CAM 5V
FTDI TX  → ESP32-CAM U0R
FTDI RX  → ESP32-CAM U0T
GPIO 0   → GND  (boot mode — only during flashing)
```

1. Connect GPIO 0 to GND
2. Power on
3. Click Upload in Arduino IDE
4. When you see `Connecting......` — press RESET button on ESP32-CAM
5. After upload — disconnect GPIO 0 from GND
6. Press RESET
7. Open Serial Monitor at **115200 baud**
8. You will see:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  BOOKMARK THIS URL: http://192.168.1.100
  This IP will NEVER change.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Using ESP32-CAM-MB

Just plug in via USB — no extra wiring needed. GPIO 0 handled automatically.

\---

## Usage

Open the IP address in any browser on the same WiFi network.

### Web UI Features

|Button|Function|
|-|-|
|▲ ▼ ◀ ▶|Pan and tilt camera|
|⌂ RETURN TO HOME|Snap back to 90°/90°|
|↻ AUTO PATROL|Continuous left-right sweep|
|NIGHT VISION|Toggle IR LED/inbuilt LED (GPIO 4)|
|MOTION DETECT|Enable/disable software motion detection|
|↺ ⊙ ↻ ↕|Rotate live feed 90°/180°/270° in browser|

### Motion Detection

When motion is detected:

* MOTION DETECTED banner appears on live feed
* Red dot pulses in header
* Clears automatically after 3 seconds of no motion

### Night Vision

* Manual: press button on web UI
* Automatic (if LDR circuit installed): hardware controlled, no button needed

\---

## 3D Printing

Files in `3d_models` folder.



### Print Settings

```
Material : PLA
Layer    : 0.2mm
Infill   : 25-30%
Walls    : 3-4 perimeters
Supports : Yes for elec\_base only
```

\---

## Troubleshooting

|Problem|Cause|Fix|
|-|-|-|
|Stream not loading|Wrong IP or not on same WiFi|Check Serial Monitor for IP|
|Camera init failed|Bad 5V supply or loose ribbon|Check LM2596S output = 5V, reseat ribbon|
|Servos jitter|Insufficient current|Connect servo power direct to LM2596S, not through breadboard|
|Only one servo moves|Wrong GPIO|Confirm GPIO 14 = pan, GPIO 15 = tilt|
|IR LED not glowing|2N2222 orientation wrong|Flat face toward you: left=E, mid=B, right=C|
|No WiFi connection|Wrong credentials or 5GHz network|ESP32 only supports 2.4GHz|
|Website laggy|WiFi power saving enabled|Already disabled in firmware — check router distance|
|Motion too sensitive|Lighting changes triggering|Increase `MOTION\_THRESHOLD` value in code|
|IP keeps changing|DHCP reassigning|Static IP already configured — check GATEWAY matches your router|

\---

## License

MIT License — free to use, modify, and distribute with attribution.

\---

*Built by Apramaeyo* 

