/*
 * ============================================================
 *  Dual-Axis CCTV Surveillance System
 *  Hardware : ESP32-CAM (AI Thinker)
 *  Features : Live MJPEG stream, Pan/Tilt servos,
 *             Motion detection, IR LED night vision
 *  Control  : Web browser over WiFi
 * ============================================================
 *
 *  WIRING SUMMARY
 *  ──────────────
 *  Pan  Servo  signal → GPIO 14   
 *  Tilt Servo  signal → GPIO 15   
 *  Both Servo  VCC   → 5V rail from buck converter
 *  Both Servo  GND   → Common GND
 *
 *  IR LED circuit:
 *    GPIO 4 → 330Ω resistor → Base of 2N2222
 *    Collector → IR LED anode
 *    IR LED cathode → GND
 *    Emitter → GND
 *  (GPIO 4 is also the onboard white flash LED — it will light
 *   up instead if you don't have IR LEDs yet, useful for testing)
 *
 *  Power:
 *    18650 pack → LM2596S → 5V → ESP32-CAM 5V pin + Servo VCC
 *    Common GND between all components
 *
 *  FLASHING INSTRUCTIONS
 *  ─────────────────────
 *  1. Install board: "AI Thinker ESP32-CAM" in Arduino IDE
 *     (add https://dl.espressif.com/dl/package_esp32_index.json
 *      to Preferences → Additional board manager URLs)
 *  2. Install library: ESP32Servo (by Kevin Harrington)
 *  3. Connect GPIO 0 to GND before powering on (boot mode)
 *  4. Select: Tools → Board → AI Thinker ESP32-CAM
 *             Tools → Port → your COM port
 *  5. Upload, then disconnect GPIO 0 from GND, press RESET
 *  6. Open Serial Monitor at 115200 baud to see the IP address
 *  7. Open that IP in your browser
 * ============================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "esp_wifi.h"       // for esp_wifi_set_ps()

// ─── WiFi credentials ────────────────────────────────────────
const char* WIFI_SSID = "WIFISSID";
const char* WIFI_PASS = "WIFIPASSWORD";

// ─── Static IP configuration ─────────────────────────────────
// Change these to match your home network.
// STATIC_IP  = the fixed address you want ESP32-CAM to always use
// GATEWAY    = your router IP (usually 192.168.1.1 or 192.168.0.1)
// SUBNET     = almost always 255.255.255.0
//
// HOW TO FIND YOUR GATEWAY:
//   Windows: open CMD → type "ipconfig" → look for "Default Gateway"
//   Android: WiFi settings → tap network → Gateway
//   After finding gateway, set STATIC_IP to same first 3 numbers
//   e.g. gateway = 192.168.1.1  →  use 192.168.1.XXX for static IP
//   Pick XXX between 100-200 to avoid conflicts with other devices
//
IPAddress STATIC_IP(192, 168, 1, XX);   // ← your chosen fixed IP
IPAddress GATEWAY  (192,168,1,XX);   // ← your router IP
IPAddress SUBNET   (255, 255, 255, 0);   // ← leave this as is
IPAddress DNS      (8,   8,   8,   8);   // ← Google DNS, leave as is

// ─── Pin definitions ─────────────────────────────────────────
#define PAN_PIN   14   // Pan servo signal
#define TILT_PIN  15   // Tilt servo signal
#define IR_PIN     4   // GPIO 4 = onboard white flash LED

// ─── Servo angle limits ──────────────────────────────────────
#define PAN_MIN   0
#define PAN_MAX   180
#define PAN_HOME  90
#define TILT_MIN  90
#define TILT_MAX  180
#define TILT_HOME 90
#define STEP      15    // target degrees per button press

// ─── Smooth servo motion config ──────────────────────────────
// Servos move via easing — accelerate, cruise, decelerate
// like a real camera gimbal. No more sudden jumps.
float panCurrent   = 90.0;   // actual current position (float for smooth steps)
float tiltCurrent  = 90.0;
float panTarget    = 90.0;   // where we want to go
float tiltTarget   = 90.0;
const float EASE_SPEED   = 0.10;  // 0.0-1.0: higher = faster/snappier
                                   // 0.10 = very smooth/slow
                                   // 0.18 = smooth but responsive  
                                   // 0.30 = fast, slight smoothing
                                   // 1.0  = instant (no easing)
const float DEAD_ZONE    = 0.4;   // stop moving when this close to target (degrees)

// ─── AI Thinker ESP32-CAM pins (OV3660 sensor variant) ──────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── Global state ────────────────────────────────────────────
Servo panServo;
Servo tiltServo;
WebServer server(80);

// panAngle/tiltAngle now track TARGET (int for JSON/display)
// Actual smooth position tracked by panCurrent/tiltCurrent (float)
int panAngle  = PAN_HOME;
int tiltAngle = TILT_HOME;
bool irOn          = false;
bool motionDetected = false;

// Software motion detection — frame differencing
uint8_t* prevFrame = nullptr;
const int MOTION_THRESHOLD = 40;   // pixel difference to count
const int MOTION_PIXELS    = 500;  // how many changed pixels = motion

// ─── Embedded Web UI ─────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CCTV Surveillance</title>
<style>
  :root {
    --bg: #f4f6f9;
    --panel: #ffffff;
    --border: #dee2e6;
    --accent: #0d6efd;
    --accent-hover: #0b5ed7;
    --text: #212529;
    --text-dim: #6c757d;
    --green: #198754;
    --red: #dc3545;
  }
  
  * { box-sizing: border-box; margin: 0; padding: 0; }
  
  body {
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 20px 16px;
    gap: 20px;
  }

  /* ── Header ── */
  header {
    width: 100%;
    max-width: 780px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid var(--border);
    padding-bottom: 16px;
  }
  .logo {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .logo-icon {
    width: 40px; height: 40px;
    background: #e9ecef;
    border-radius: 8px;
    display: flex; align-items: center; justify-content: center;
    font-size: 13px;
    font-weight: 700;
    color: var(--accent);
  }
  .logo-text { font-size: 14px; font-weight: 600; letter-spacing: 0.05em; color: var(--text); }
  .logo-sub  { font-size: 11px; color: var(--text-dim); letter-spacing: 0.05em; margin-top: 2px;}
  .status-bar {
    display: flex; gap: 16px; font-size: 12px; font-weight: 500; color: var(--text-dim);
  }
  .status-item { display: flex; align-items: center; gap: 6px; }
  .dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--green);
    animation: pulse 2s infinite;
  }
  .dot.off { background: #ced4da; animation: none; }
  .dot.alert { background: var(--red); }
  @keyframes pulse {
    0%,100% { opacity: 1; } 50% { opacity: 0.4; }
  }

  /* ── Stream panel ── */
  .stream-panel {
    width: 100%;
    max-width: 780px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    overflow: hidden;
    position: relative;
    box-shadow: 0 2px 4px rgba(0,0,0,0.02);
  }
  .stream-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.05em;
    color: var(--text-dim);
  }
  .rec-badge {
    display: flex; align-items: center; gap: 6px;
    color: var(--red); font-weight: 700;
  }
  .rec-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--red);
    animation: pulse 1s infinite;
  }
  
  .motion-overlay {
    position: absolute;
    bottom: 45px; left: 0; right: 0;
    display: flex; justify-content: center;
    pointer-events: none;
  }
  .motion-badge {
    background: var(--red);
    color: #fff;
    padding: 6px 16px;
    border-radius: 20px;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.05em;
    display: none;
    box-shadow: 0 2px 8px rgba(220, 53, 69, 0.4);
  }
  .motion-badge.visible { display: block; }
  .stream-footer {
    padding: 10px 16px;
    display: flex; justify-content: space-between;
    font-size: 11px; font-weight: 500; color: var(--text-dim);
    border-top: 1px solid var(--border);
  }

  /* ── Controls ── */
  .controls-row {
    width: 100%;
    max-width: 780px;
    display: flex;
    gap: 20px;
    flex-wrap: wrap;
  }
  .ctrl-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 20px;
    flex: 1;
    min-width: 280px;
    box-shadow: 0 2px 4px rgba(0,0,0,0.02);
  }
  .ctrl-label {
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.05em;
    color: var(--text-dim);
    margin-bottom: 16px;
    text-transform: uppercase;
  }

  /* D-pad */
  .dpad {
    display: grid;
    grid-template-columns: repeat(3, 54px);
    grid-template-rows: repeat(3, 54px);
    gap: 8px;
    justify-content: center;
  }
  .dpad-btn {
    background: #f8f9fa;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    font-size: 16px;
    font-weight: bold;
    cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    transition: all 0.15s ease;
    user-select: none;
    -webkit-tap-highlight-color: transparent;
  }
  .dpad-btn:hover {
    background: #e9ecef;
  }
  .dpad-btn:active, .dpad-btn.pressed {
    background: #dde2e6;
    transform: scale(0.95);
  }
  .dpad-center {
    background: #e9ecef;
    color: var(--text-dim);
    font-size: 11px;
    font-weight: 600;
    cursor: default;
  }
  .dpad-center:active { transform: none; }

  /* Angle readout */
  .angle-display {
    margin-top: 16px;
    display: flex; justify-content: center; gap: 24px;
    font-size: 12px; color: var(--text-dim);
    text-align: center;
  }
  .angle-val { color: var(--text); font-size: 18px; font-weight: 700; }

  /* Toggle buttons */
  .toggle-group {
    display: flex; flex-direction: column; gap: 12px;
  }

  .toggle-btn {
    background: #f8f9fa;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    padding: 14px 16px;
    cursor: pointer;
    font-family: inherit;
    font-size: 13px;
    font-weight: 500;
    display: flex; align-items: center; justify-content: space-between;
    transition: all 0.2s ease;
  }
  .toggle-btn:hover { border-color: var(--accent); }
  .toggle-btn.active {
    border-color: var(--accent);
    background: #f0f7ff;
    color: var(--accent);
  }
  .toggle-indicator {
    width: 32px; height: 18px;
    background: #ced4da;
    border-radius: 10px;
    position: relative;
    transition: background 0.2s;
  }
  .toggle-indicator::after {
    content: '';
    position: absolute;
    width: 14px; height: 14px;
    border-radius: 50%;
    background: #fff;
    top: 2px; left: 2px;
    transition: transform 0.2s, background 0.2s;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
  }
  .toggle-btn.active .toggle-indicator { background: var(--accent); }
  .toggle-btn.active .toggle-indicator::after {
    transform: translateX(14px);
  }

  /* ── Big Motion Detect Button ── */
  .motion-ctrl-wrap {
    margin-top: 20px;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .motion-ctrl-label {
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.05em;
    color: var(--text-dim);
    text-transform: uppercase;
  }
  .motion-big-btn {
    width: 100%;
    border: none;
    border-radius: 6px;
    cursor: pointer;
    font-family: inherit;
    padding: 0;
    overflow: hidden;
    position: relative;
    transition: transform 0.1s;
    outline: none;
  }
  .motion-big-btn:active { transform: scale(0.98); }

  /* Inner layout */
  .motion-btn-inner {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 16px;
    transition: all 0.3s ease;
    border: 1px solid transparent;
    border-radius: 6px;
  }

  /* OFF state */
  .motion-big-btn.off .motion-btn-inner {
    background: #f8f9fa;
    border-color: var(--border);
  }
  .motion-big-btn.off .motion-btn-text  { color: var(--text-dim); font-size: 13px; font-weight: 500; }
  .motion-big-btn.off .motion-btn-state {
    background: #e9ecef;
    color: var(--text-dim);
    font-size: 11px;
    font-weight: 600;
    padding: 4px 12px;
    border-radius: 4px;
  }

  /* ON state */
  .motion-big-btn.on .motion-btn-inner {
    background: #f0fdf4;
    border-color: #a3e6cd;
  }
  .motion-big-btn.on .motion-btn-text  { color: var(--green); font-size: 13px; font-weight: 500; }
  .motion-big-btn.on .motion-btn-state {
    background: #d1f2e2;
    color: var(--green);
    font-size: 11px;
    font-weight: 600;
    padding: 4px 12px;
    border-radius: 4px;
  }

  /* Confirm flash overlay */
  .motion-big-btn::after {
    content: '';
    position: absolute;
    inset: 0;
    background: rgba(0,0,0,0.05);
    opacity: 0;
    pointer-events: none;
    transition: opacity 0.15s;
  }
  .motion-big-btn.flash::after { opacity: 1; }

  /* Sensitivity slider */
  .sensitivity-row {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 8px 4px 4px;
    opacity: 1;
    transition: opacity 0.3s;
  }
  .sensitivity-row.hidden { opacity: 0.3; pointer-events: none; }
  .sens-label { font-size: 12px; font-weight: 500; color: var(--text-dim); white-space: nowrap; }
  .sens-slider {
    flex: 1;
    -webkit-appearance: none;
    height: 4px;
    border-radius: 2px;
    background: #e9ecef;
    outline: none;
    cursor: pointer;
  }
  .sens-slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 16px; height: 16px;
    border-radius: 50%;
    background: var(--accent);
    cursor: pointer;
    transition: background 0.2s;
  }
  .sens-slider::-webkit-slider-thumb:hover { background: var(--accent-hover); }
  .sens-val {
    font-size: 12px;
    font-weight: 600;
    color: var(--text);
    min-width: 24px;
    text-align: right;
  }

  /* Home button */
  .home-btn {
    width: 100%;
    background: #f8f9fa;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    padding: 12px;
    cursor: pointer;
    font-family: inherit;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.05em;
    margin-top: 16px;
    transition: all 0.2s ease;
  }
  .home-btn:hover { border-color: var(--accent); color: var(--accent); background: #f0f7ff; }

  /* Rotate / patrol button */
  .rotate-btn {
    width: 100%;
    border-radius: 6px;
    padding: 12px;
    cursor: pointer;
    font-family: inherit;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.05em;
    margin-top: 10px;
    border: 1px solid var(--border);
    background: #f8f9fa;
    color: var(--text);
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    transition: all 0.2s ease;
  }
  .rotate-btn.spinning {
    border-color: var(--accent);
    color: var(--accent);
    background: #f0f7ff;
  }
  .spin-icon {
    display: inline-block;
    transition: transform 0.3s;
    font-size: 14px;
  }
  .rotate-btn.spinning .spin-icon {
    animation: spinIcon 1.5s linear infinite;
  }
  @keyframes spinIcon {
    from { transform: rotate(0deg); }
    to   { transform: rotate(360deg); }
  }

  /* Stream image rotation */
  .stream-img {
    width: 100%;
    display: block;
    aspect-ratio: 4/3;
    object-fit: cover;
    background: #e9ecef;
    transition: transform 0.4s cubic-bezier(0.4, 0, 0.2, 1);
    transform-origin: center center;
  }
  .stream-img.rot90  { transform: rotate(90deg)  scale(0.75); }
  .stream-img.rot180 { transform: rotate(180deg) scale(1.0);  }
  .stream-img.rot270 { transform: rotate(270deg) scale(0.75); }

  /* Rotate feed button row */
  .feed-rotate-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 12px 16px;
    border-top: 1px solid var(--border);
    background: #f8f9fa;
  }
  .feed-rotate-label {
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.05em;
    color: var(--text-dim);
  }
  .feed-rotate-btns {
    display: flex;
    gap: 8px;
  }
  .feed-rot-btn {
    background: #ffffff;
    border: 1px solid var(--border);
    border-radius: 4px;
    color: var(--text);
    padding: 6px 12px;
    font-size: 12px;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.15s ease;
    font-family: inherit;
  }
  .feed-rot-btn:hover  { border-color: var(--accent); color: var(--accent); }
  .feed-rot-btn:active { background: #e9ecef; transform: scale(0.95); }
  .feed-rot-btn.active { border-color: var(--accent); color: var(--accent); background: #f0f7ff; font-weight: 600;}
  .rot-angle-badge {
    font-size: 12px;
    font-weight: 600;
    color: var(--text);
    min-width: 30px;
    text-align: right;
  }

  /* Footer */
  footer {
    font-size: 11px;
    font-weight: 500;
    color: var(--text-dim);
    letter-spacing: 0.05em;
    margin-top: 10px;
  }

  @media (max-width: 520px) {
    .status-bar { display: none; }
  }
</style>
</head>
<body>

<header>
  <div class="logo">
    <div class="logo-icon">CAM</div>
    <div>
      <div class="logo-text">SURVEILLANCE SYSTEM</div>
      <div class="logo-sub">ESP32-CAM · DUAL AXIS</div>
    </div>
  </div>
  <div class="status-bar">
    <div class="status-item"><div class="dot" id="streamDot"></div>STREAM</div>
    <div class="status-item"><div class="dot off" id="irDot"></div>IR</div>
    <div class="status-item"><div class="dot off" id="motionDot"></div>MOTION</div>
  </div>
</header>

<!-- Stream -->
<div class="stream-panel">
  <div class="stream-header">
    <span>CAM-01 · LIVE</span>
    <div class="rec-badge"><div class="rec-dot"></div>REC</div>
    <span id="timeDisplay">--:--:--</span>
  </div>
  <img class="stream-img" id="streamImg" src="/stream" alt="Live Feed">
  <div class="motion-overlay">
    <div class="motion-badge" id="motionBadge">MOTION DETECTED</div>
  </div>
  <div class="stream-footer">
    <span>PAN <span id="panReadout">90</span>°</span>
    <span>TILT <span id="tiltReadout">90</span>°</span>
    <span id="fpsDisplay">-- FPS</span>
  </div>
  <div class="feed-rotate-row">
    <span class="feed-rotate-label">ROTATE FEED</span>
    <div class="feed-rotate-btns">
      <button class="feed-rot-btn" onclick="rotateFeed(-90)" title="Rotate left 90°">-90°</button>
      <button class="feed-rot-btn" onclick="rotateFeed(0)"   title="Reset rotation" id="rotResetBtn">0°</button>
      <button class="feed-rot-btn" onclick="rotateFeed(90)"  title="Rotate right 90°">+90°</button>
      <button class="feed-rot-btn" onclick="rotateFeed(180)" title="Flip 180°">180°</button>
    </div>
    <span class="rot-angle-badge" id="rotBadge">0°</span>
  </div>
</div>

<!-- Controls -->
<div class="controls-row">

  <!-- D-pad -->
  <div class="ctrl-panel">
    <div class="ctrl-label">PAN / TILT CONTROL</div>
    <div class="dpad">
      <div></div>
      <div class="dpad-btn" onclick="move('up')"    id="btn-up">↑</div>
      <div></div>
      <div class="dpad-btn" onclick="move('left')"  id="btn-left">←</div>
      <div class="dpad-btn dpad-center">CAM</div>
      <div class="dpad-btn" onclick="move('right')" id="btn-right">→</div>
      <div></div>
      <div class="dpad-btn" onclick="move('down')"  id="btn-down">↓</div>
      <div></div>
    </div>
    <div class="angle-display">
      <div>PAN <br><span class="angle-val" id="panAngle">90</span>°</div>
      <div>TILT<br><span class="angle-val" id="tiltAngle">90</span>°</div>
    </div>
    <button class="home-btn" onclick="goHome()">RETURN TO HOME</button>
    <button class="rotate-btn" id="rotateBtn" onclick="togglePatrol()">
      <span class="spin-icon">O</span>
      <span id="rotateBtnText">AUTO PATROL</span>
    </button>
  </div>

  <!-- Toggles -->
  <div class="ctrl-panel">
    <div class="ctrl-label">SYSTEM CONTROLS</div>
    <div class="toggle-group">

      <button class="toggle-btn" id="irBtn" onclick="toggleIR()">
        <span>NIGHT VISION</span>
        <div class="toggle-indicator"></div>
      </button>

    </div>

    <!-- Big Motion Detect Button -->
    <div class="motion-ctrl-wrap">
      <div class="motion-ctrl-label">MOTION DETECTION</div>

      <button class="motion-big-btn on" id="motionBtn" onclick="toggleMotion()">
        <div class="motion-btn-inner">
          <span class="motion-btn-text" id="motionBtnText">MOTION SENSING ON</span>
          <span class="motion-btn-state" id="motionBtnState">ON</span>
        </div>
      </button>

      <!-- PIR sensor -->
      <div style="font-size:11px; font-weight:500; color:var(--text-dim); margin-top:8px; letter-spacing:0.02em;">
        SOFTWARE DETECTION
      </div>
    </div>

    <div style="margin-top:20px; font-size:12px; font-weight:500; color:var(--text-dim); line-height:1.8;">
      <div style="font-weight:600; font-size: 11px; letter-spacing: 0.05em; margin-bottom: 8px;">SYSTEM STATUS</div>
      <div>
        WiFi <span style="float: right; font-weight: 600; color:var(--green)">CONNECTED</span><br>
        Stream <span style="float: right; font-weight: 600; color:var(--green)">ACTIVE</span><br>
        IR LEDs <span id="irStatus" style="float: right; font-weight: 600; color:var(--text-dim)">OFF</span><br>
        Motion Detect <span id="mdStatus" style="float: right; font-weight: 600; color:var(--green)">ON</span>
      </div>
    </div>
  </div>

</div>

<footer>ESP32-CAM SURVEILLANCE v1.0 · APRAMAEYO</footer>

<script>
  let pan = 90, tilt = 90;
  let irActive = false;
  let motionActive = true;
  let lastMotion = 0;

  // Clock
  function updateClock() {
    document.getElementById('timeDisplay').textContent =
      new Date().toLocaleTimeString('en-IN', {hour12: false});
  }
  setInterval(updateClock, 1000);
  updateClock();

  // Button flash feedback
  function flashBtn(id) {
    const el = document.getElementById('btn-' + id);
    if (!el) return;
    el.classList.add('pressed');
    setTimeout(() => el.classList.remove('pressed'), 150);
  }

  // Move servo
  function move(dir) {
    flashBtn(dir);
    fetch('/move?dir=' + dir)
      .then(r => r.json())
      .then(d => {
        pan = d.pan; tilt = d.tilt;
        document.getElementById('panAngle').textContent  = pan;
        document.getElementById('tiltAngle').textContent = tilt;
        document.getElementById('panReadout').textContent  = pan;
        document.getElementById('tiltReadout').textContent = tilt;
      });
  }

  // Go home
  function goHome() {
    fetch('/home')
      .then(r => r.json())
      .then(d => {
        pan = d.pan; tilt = d.tilt;
        document.getElementById('panAngle').textContent  = pan;
        document.getElementById('tiltAngle').textContent = tilt;
        document.getElementById('panReadout').textContent  = pan;
        document.getElementById('tiltReadout').textContent = tilt;
      });
  }

  // Toggle IR
  function toggleIR() {
    fetch('/ir?state=' + (irActive ? '0' : '1'))
      .then(r => r.json())
      .then(d => {
        irActive = d.ir;
        document.getElementById('irBtn').classList.toggle('active', irActive);
        document.getElementById('irDot').classList.toggle('off', !irActive);
        document.getElementById('irStatus').textContent = irActive ? 'ON' : 'OFF';
        document.getElementById('irStatus').style.color = irActive ? 'var(--accent)' : 'var(--text-dim)';
      });
  }

  // Toggle PIR motion detection on/off
  function toggleMotion() {
    const btn      = document.getElementById('motionBtn');
    const btnText  = document.getElementById('motionBtnText');
    const btnState = document.getElementById('motionBtnState');
    const mdStatus = document.getElementById('mdStatus');
    const motionDot = document.getElementById('motionDot');

    fetch('/motion_enable?state=' + (motionActive ? '0' : '1'))
      .then(r => r.json())
      .then(d => {
        motionActive = d.enabled;
        btn.classList.toggle('on',  motionActive);
        btn.classList.toggle('off', !motionActive);
        btnText.textContent  = motionActive ? 'MOTION SENSING ON' : 'MOTION SENSING OFF';
        btnState.textContent = motionActive ? 'ON' : 'OFF';
        mdStatus.textContent = motionActive ? 'ON' : 'OFF';
        mdStatus.style.color = motionActive ? 'var(--green)' : 'var(--text-dim)';
        if (!motionActive) {
          motionDot.classList.add('off');
          motionDot.classList.remove('alert');
          document.getElementById('motionBadge').classList.remove('visible');
        } else {
          motionDot.classList.remove('off');
        }
        btn.classList.add('flash');
        setTimeout(() => btn.classList.remove('flash'), 200);
      });
  }

  // Feed rotation — pure CSS, no server needed
  let feedRotation = 0;
  function rotateFeed(deg) {
    const img = document.getElementById('streamImg');
    if (deg === 0) {
      feedRotation = 0;
    } else {
      feedRotation = ((feedRotation + deg) % 360 + 360) % 360;
    }
    // Remove all rotation classes
    img.classList.remove('rot90', 'rot180', 'rot270');
    // Apply correct class
    if      (feedRotation === 90)  img.classList.add('rot90');
    else if (feedRotation === 180) img.classList.add('rot180');
    else if (feedRotation === 270) img.classList.add('rot270');

    // Update badge
    document.getElementById('rotBadge').textContent = feedRotation + '°';

    // Highlight reset button when rotated
    document.getElementById('rotResetBtn').classList.toggle('active', feedRotation !== 0);
  }

  // Auto patrol toggle
  let patrolActive = false;
  function togglePatrol() {
    patrolActive = !patrolActive;
    const btn     = document.getElementById('rotateBtn');
    const btnText = document.getElementById('rotateBtnText');
    fetch('/patrol?state=' + (patrolActive ? '1' : '0'))
      .then(r => r.json())
      .then(d => {
        patrolActive = d.patrol;
        btn.classList.toggle('spinning', patrolActive);
        btnText.textContent = patrolActive ? 'STOP PATROL' : 'AUTO PATROL';
        // Disable d-pad while patrolling
        ['up','down','left','right'].forEach(id => {
          const el = document.getElementById('btn-' + id);
          if (el) el.style.opacity = patrolActive ? '0.3' : '1';
        });
      });
  }

  // Poll motion status every 1.5s
  function pollMotion() {
    fetch('/status')
      .then(r => r.json())
      .then(d => {
        const badge = document.getElementById('motionBadge');
        const dot   = document.getElementById('motionDot');
        if (d.motion) {
          lastMotion = Date.now();
          badge.classList.add('visible');
          dot.classList.add('alert');
          dot.classList.remove('off');
        } else if (Date.now() - lastMotion > 3000) {
          badge.classList.remove('visible');
          dot.classList.remove('alert');
          dot.classList.add('off');
        }
      })
      .catch(() => {});
  }
  setInterval(pollMotion, 800);
</script>
</body>
</html>
)rawhtml";

// ─── Camera init ─────────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Low-latency stream config:
  // QVGA = smaller frames = faster WiFi transfer = less lag
  // jpeg_quality 10 = good quality, small file (lower = better quality)
  // fb_count 3 = triple buffer — camera fills next frame while we transmit current
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240 — optimal for low-lag WiFi stream
    config.jpeg_quality = 10;              // 0-63, lower = better quality
    config.fb_count     = 3;              // triple buffer — no wait between frames
    config.grab_mode    = CAMERA_GRAB_LATEST; // always grab freshest frame — kills lag
  } else {
    config.frame_size   = FRAMESIZE_QQVGA; // 160×120 — minimum lag, no PSRAM
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  // Image settings for OV3660
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 1);     // OV3660 needs +1 brightness to avoid dark image
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);        // 0=auto white balance
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);           // enable AEC DSP — important for OV3660
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);        // set to 1 if image is mirrored
  s->set_vflip(s, 1);          // OV3660 is mounted inverted — flip vertically
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  return true;
}

// ─── Smooth servo easing ─────────────────────────────────────
// Called every loop(). Uses exponential easing (lerp):
//   new_pos = current + (target - current) * EASE_SPEED
// This creates natural acceleration/deceleration automatically.
// The servo slows down as it approaches the target — no sudden stops.
void smoothServoTick() {
  bool panMoved  = false;
  bool tiltMoved = false;

  // Pan easing
  float panDiff = panTarget - panCurrent;
  if (fabs(panDiff) > DEAD_ZONE) {
    panCurrent += panDiff * EASE_SPEED;
    panCurrent  = constrain(panCurrent, PAN_MIN, PAN_MAX);
    panServo.write((int)panCurrent);
    panMoved = true;
  }

  // Tilt easing
  float tiltDiff = tiltTarget - tiltCurrent;
  if (fabs(tiltDiff) > DEAD_ZONE) {
    tiltCurrent += tiltDiff * EASE_SPEED;
    tiltCurrent  = constrain(tiltCurrent, TILT_MIN, TILT_MAX);
    tiltServo.write((int)tiltCurrent);
    tiltMoved = true;
  }
}

// ─── Auto patrol (continuous pan sweep) ─────────────────────
bool patrolActive  = false;
int  patrolDir     = 1;       // 1 = moving right, -1 = moving left
unsigned long lastPatrolMove = 0;
const int PATROL_STEP_MS = 80; // ms between each 1° step — lower = faster sweep

void patrolTick() {
  if (!patrolActive) return;
  unsigned long now = millis();
  if (now - lastPatrolMove < PATROL_STEP_MS) return;
  lastPatrolMove = now;

  // Move target — easing system handles the actual smooth movement
  panTarget += patrolDir * 0.5;  // 0.5° steps for ultra-smooth patrol sweep
  if (panTarget >= PAN_MAX) { panTarget = PAN_MAX; patrolDir = -1; }
  if (panTarget <= PAN_MIN) { panTarget = PAN_MIN; patrolDir =  1; }
  panAngle = (int)panTarget;
}

void handlePatrol() {
  String state = server.arg("state");
  patrolActive = (state == "1");
  if (!patrolActive) {
    // snap back to home when patrol stops
    panAngle = PAN_HOME;
    panServo.write(PAN_HOME);
  }
  String json = "{\"patrol\":" + String(patrolActive ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

// ─── Software motion detection (frame differencing) ──────────
bool motionEnabled = true;

bool checkMotion(camera_fb_t* fb) {
  if (!motionEnabled) return false;

  if (prevFrame == nullptr) {
    prevFrame = (uint8_t*)malloc(fb->len);
    if (prevFrame) memcpy(prevFrame, fb->buf, fb->len);
    return false;
  }

  int diffCount = 0;
  int step = fb->len / 2000;
  if (step < 1) step = 1;

  for (size_t i = 100; i < fb->len - 100; i += step) {
    int diff = abs((int)fb->buf[i] - (int)prevFrame[i]);
    if (diff > MOTION_THRESHOLD) diffCount++;
    if (diffCount > MOTION_PIXELS) break;
  }

  memcpy(prevFrame, fb->buf, fb->len);
  return diffCount > MOTION_PIXELS;
}

// ─── HTTP handlers ───────────────────────────────────────────

// Root — serve web UI
void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

// MJPEG stream — runs on its own FreeRTOS task so it never
// blocks the web server, which is why pan/tilt controls were laggy before
WiFiClient streamClient;
bool streamActive = false;

void streamTask(void* pvParameters) {
  // Send headers
  streamClient.print("HTTP/1.1 200 OK\r\n");
  streamClient.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  streamClient.print("Cache-Control: no-cache, no-store, must-revalidate\r\n");
  streamClient.print("Pragma: no-cache\r\n");
  streamClient.print("Access-Control-Allow-Origin: *\r\n\r\n");

  while (streamClient.connected()) {

    // ── FLUSH stale frames from buffer ─────────────────────
    // With CAMERA_GRAB_LATEST + fb_count=3, we must discard
    // any queued frames to always send the freshest one
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

    // If another frame is already waiting, return this one and grab again
    camera_fb_t* fb2 = esp_camera_fb_get();
    if (fb2) {
      esp_camera_fb_return(fb);  // discard older frame
      fb = fb2;                  // use newer frame
    }

    motionDetected = checkMotion(fb);

    // ── Send frame header ───────────────────────────────────
    char hdr[80];
    int hlen = snprintf(hdr, sizeof(hdr),
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      (unsigned)fb->len);
    streamClient.write((uint8_t*)hdr, hlen);

    // ── Send frame data in one large write ──────────────────
    // Single write() is faster than chunked — WiFi stack batches it
    size_t written = streamClient.write(fb->buf, fb->len);
    streamClient.write((uint8_t*)"\r\n", 2);
    esp_camera_fb_return(fb);

    // ── No delay — run as fast as WiFi allows ───────────────
    // Task yield only — lets other tasks breathe without sleeping
    taskYIELD();
  }

  streamActive = false;
  vTaskDelete(NULL);
}

void handleStream() {
  if (streamActive) {
    server.send(503, "text/plain", "Stream already active");
    return;
  }
  streamClient = server.client();
  streamActive = true;
  // Priority 5 = high priority, Core 0 = dedicated core
  // Web server runs on Core 1 — completely separate, zero contention
  xTaskCreatePinnedToCore(streamTask, "stream", 8192, NULL, 5, NULL, 0);
}

// Servo move
void handleMove() {
  String dir = server.arg("dir");

  if      (dir == "left")  panAngle  = max(PAN_MIN,  panAngle  - STEP);
  else if (dir == "right") panAngle  = min(PAN_MAX,  panAngle  + STEP);
  else if (dir == "up")    tiltAngle = min(TILT_MAX, tiltAngle + STEP);
  else if (dir == "down")  tiltAngle = max(TILT_MIN, tiltAngle - STEP);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  String json = "{\"pan\":" + String(panAngle) +
                ",\"tilt\":" + String(tiltAngle) + "}";
  server.send(200, "application/json", json);
}

// Return to home position
void handleHome() {
  panAngle  = PAN_HOME;
  tiltAngle = TILT_HOME;
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  String json = "{\"pan\":" + String(panAngle) +
                ",\"tilt\":" + String(tiltAngle) + "}";
  server.send(200, "application/json", json);
}

// IR LED toggle
void handleIR() {
  String state = server.arg("state");
  irOn = (state == "1");
  digitalWrite(IR_PIN, irOn ? HIGH : LOW);
  String json = "{\"ir\":" + String(irOn ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

// Motion enable toggle
void handleMotionEnable() {
  String state = server.arg("state");
  motionEnabled = (state == "1");
  if (!motionEnabled) motionDetected = false;
  String json = "{\"enabled\":" + String(motionEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}
// Status poll
void handleStatus() {
  String json = "{\"motion\":" + String(motionDetected ? "true" : "false") +
                ",\"ir\":"     + String(irOn ? "true" : "false") +
                ",\"pan\":"    + String(panAngle) +
                ",\"tilt\":"   + String(tiltAngle) + "}";
  server.send(200, "application/json", json);
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32-CAM Surveillance System starting...");

  // IR LED pin
  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, LOW);



  // Servos
  panServo.attach(PAN_PIN,  500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  panCurrent  = PAN_HOME;   tiltCurrent  = TILT_HOME;
  panTarget   = PAN_HOME;   tiltTarget   = TILT_HOME;
  panServo.write(PAN_HOME); tiltServo.write(TILT_HOME);
  Serial.println("Servos initialized at home position (smooth easing ON).");

  // Camera
  if (!initCamera()) {
    Serial.println("Camera init failed! Check wiring and restart.");
    while (true) delay(1000);
  }
  Serial.println("Camera initialized.");

  // WiFi — static IP so address never changes
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // CRITICAL — disables WiFi modem sleep
                                  // Default sleep mode adds 50-200ms latency per packet
  esp_wifi_set_ps(WIFI_PS_NONE); // ESP-IDF level power save off — belt AND suspenders
  if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS)) {
    Serial.println("Static IP config failed — check GATEWAY address");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed! Check SSID/password. Restarting...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("\nWiFi connected!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print  ("  BOOKMARK THIS URL: http://");
  Serial.println(WiFi.localIP());
  Serial.println("  This IP will NEVER change.");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // Routes
  server.on("/",              handleRoot);
  server.on("/stream",        handleStream);
  server.on("/move",          handleMove);
  server.on("/home",          handleHome);
  server.on("/ir",            handleIR);
  server.on("/motion_enable", handleMotionEnable);
  server.on("/patrol",        handlePatrol);
  server.on("/status",        handleStatus);
  server.begin();
  Serial.println("Web server started.");
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  smoothServoTick();
  patrolTick();

  delay(1);
}
