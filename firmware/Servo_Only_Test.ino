/*
 * ============================================================
 *  SERVO MOTOR TEST — ESP32-CAM (AI Thinker)
 *  Tests pan + tilt MG90S servos ONLY
 *  No camera, no WiFi, no other components needed
 *
 *  MINIMUM WIRING TO RUN THIS:
 *
 *  Power (from LM2596S set to 5V):
 *    5V rail  →  ESP32-CAM 5V pin
 *    5V rail  →  Both servo VCC 
 *    GND rail →  ESP32-CAM GND pin
 *    GND rail →  Both servo GND 
 *
 *  Pan Servo (horizontal):
 *    Signal  → GPIO 14
 *
 *  Tilt Servo (vertical):
 *    Signal  → GPIO 15
 *
 *  LIBRARY NEEDED:
 *    ESP32Servo by Kevin Harrington
 *    Install via: Sketch → Include Library → Manage Libraries
 *    Search "ESP32Servo" → Install
 * ============================================================
 */

#include <ESP32Servo.h>

#define PAN_PIN   14
#define TILT_PIN  15

Servo panServo;
Servo tiltServo;

// ── Helper: move servo slowly step by step ──────────────────
void slowMove(Servo &servo, int fromAngle, int toAngle, int stepDelay = 15) {
  if (fromAngle < toAngle) {
    for (int a = fromAngle; a <= toAngle; a++) {
      servo.write(a);
      delay(stepDelay);
    }
  } else {
    for (int a = fromAngle; a >= toAngle; a--) {
      servo.write(a);
      delay(stepDelay);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("  SERVO TEST — ESP32-CAM");
  Serial.println("====================================");
  Serial.println();

  // Attach servos
  panServo.attach(PAN_PIN,   500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  Serial.println("[OK] Servos attached");
  Serial.println("     Pan  → GPIO " + String(PAN_PIN));
  Serial.println("     Tilt → GPIO " + String(TILT_PIN));
  Serial.println();

  // ── TEST 1: Center both servos ──────────────────────────
  Serial.println("TEST 1 — CENTER (both to 90°)");
  panServo.write(90);
  tiltServo.write(90);
  Serial.println("  Both servos → 90° (home position)");
  Serial.println("  Both should be pointing straight ahead");
  delay(2000);

  // ── TEST 2: Pan servo only ──────────────────────────────
  Serial.println();
  Serial.println("TEST 2 — PAN SERVO ONLY (GPIO 14)");
  Serial.println("  Tilt stays at 90° — only pan moves");
  Serial.println();

  Serial.println("  Pan: 90° → 0° (full left)");
  slowMove(panServo, 90, 0);
  delay(800);

  Serial.println("  Pan: 0° → 180° (full right)");
  slowMove(panServo, 0, 180);
  delay(800);

  Serial.println("  Pan: 180° → 90° (back to center)");
  slowMove(panServo, 180, 90);
  delay(1000);

  Serial.println("  [CHECK] Did the pan servo sweep left then right?");
  Serial.println("  If YES → Pan servo working");
  Serial.println("  If NO  → Check GPIO 14 wiring and 5V power");

  delay(2000);

  // ── TEST 3: Tilt servo only ─────────────────────────────
  Serial.println();
  Serial.println("TEST 3 — TILT SERVO ONLY (GPIO 15)");
  Serial.println("  Pan stays at 90° — only tilt moves");
  Serial.println();

  Serial.println("  Tilt: 90° → 150° (tilt up)");
  slowMove(tiltServo, 90, 150);
  delay(800);

  Serial.println("  Tilt: 150° → 30° (tilt down)");
  slowMove(tiltServo, 150, 30);
  delay(800);

  Serial.println("  Tilt: 30° → 90° (back to center)");
  slowMove(tiltServo, 30, 90);
  delay(1000);

  Serial.println("  [CHECK] Did the tilt servo go up then down?");
  Serial.println("  If YES → Tilt servo working");
  Serial.println("  If NO  → Check GPIO 15 wiring and 5V power");

  delay(2000);

  // ── TEST 4: Both together ───────────────────────────────
  Serial.println();
  Serial.println("TEST 4 — BOTH SERVOS TOGETHER");

  Serial.println("  Moving to top-left corner...");
  panServo.write(0);
  tiltServo.write(150);
  delay(1500);

  Serial.println("  Moving to top-right corner...");
  panServo.write(180);
  tiltServo.write(150);
  delay(1500);

  Serial.println("  Moving to bottom-right corner...");
  panServo.write(180);
  tiltServo.write(30);
  delay(1500);

  Serial.println("  Moving to bottom-left corner...");
  panServo.write(0);
  tiltServo.write(30);
  delay(1500);

  Serial.println("  Returning to HOME...");
  panServo.write(90);
  tiltServo.write(90);
  delay(1500);

  // ── TEST 5: Nod and shake ───────────────────────────────
  Serial.println();
  Serial.println("TEST 5 — NOD (tilt) AND SHAKE (pan)");

  Serial.println("  Nodding 3 times (tilt up/down)...");
  for (int i = 0; i < 3; i++) {
    tiltServo.write(115);
    delay(300);
    tiltServo.write(65);
    delay(300);
  }
  tiltServo.write(90);
  delay(500);

  Serial.println("  Shaking 3 times (pan left/right)...");
  for (int i = 0; i < 3; i++) {
    panServo.write(60);
    delay(300);
    panServo.write(120);
    delay(300);
  }
  panServo.write(90);
  delay(500);

  // ── DONE ────────────────────────────────────────────────
  Serial.println();
  Serial.println("====================================");
  Serial.println("  ALL SERVO TESTS COMPLETE");
  Serial.println("====================================");
  Serial.println();
  Serial.println("Both servos now hold at 90° (home).");
  Serial.println();
  Serial.println("RESULTS:");
  Serial.println("  If pan moved  → GPIO 14 wiring correct");
  Serial.println("  If tilt moved → GPIO 15 wiring correct");
  Serial.println("  If neither moved:");
  Serial.println("    1. Check 5V on servo VCC (red) wires");
  Serial.println("    2. Check GND on servo GND (brown) wires");
  Serial.println("    3. Check signal wires on GPIO 14 and 15");
  Serial.println("    4. Confirm ESP32Servo library is installed");
  Serial.println();
  Serial.println("Type any letter + Enter in Serial Monitor");
  Serial.println("to run a single servo sweep again:");
  Serial.println("  p = pan sweep");
  Serial.println("  t = tilt sweep");
  Serial.println("  b = both corners");
  Serial.println("  h = go home (90/90)");
}

void loop() {
  // Interactive control via Serial Monitor after tests
  if (Serial.available()) {
    char c = tolower(Serial.read());

    if (c == 'p') {
      Serial.println("Pan sweep...");
      slowMove(panServo, 90, 0);
      delay(300);
      slowMove(panServo, 0, 180);
      delay(300);
      slowMove(panServo, 180, 90);
      Serial.println("Pan done.");

    } else if (c == 't') {
      Serial.println("Tilt sweep...");
      slowMove(tiltServo, 90, 150);
      delay(300);
      slowMove(tiltServo, 150, 30);
      delay(300);
      slowMove(tiltServo, 30, 90);
      Serial.println("Tilt done.");

    } else if (c == 'b') {
      Serial.println("Corner tour...");
      panServo.write(0);   tiltServo.write(150); delay(1000);
      panServo.write(180); tiltServo.write(150); delay(1000);
      panServo.write(180); tiltServo.write(30);  delay(1000);
      panServo.write(0);   tiltServo.write(30);  delay(1000);
      panServo.write(90);  tiltServo.write(90);  delay(500);
      Serial.println("Done.");

    } else if (c == 'h') {
      Serial.println("Going home (90/90)...");
      panServo.write(90);
      tiltServo.write(90);
      Serial.println("Done.");
    }
  }
}
