#include <AFMotor.h>

#define BUZZER_PIN 9

// Pan stepper on M1 and M2 (200 steps/rev, shield port 1)
AF_Stepper panStepper(200, 1);
// Tilt stepper on M3 and M4 (200 steps/rev, shield port 2)
AF_Stepper tiltStepper(200, 2);

void setup() {
  Serial.begin(115200);
  // Set stepper speeds (RPM)
  panStepper.setSpeed(600);
  tiltStepper.setSpeed(100);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("PAN")) {
      int steps = cmd.substring(3).toInt();
      int dir = steps >= 0 ? FORWARD : BACKWARD;
      int count = steps >= 0 ? steps : -steps;
      panStepper.step(count, dir, MICROSTEP);
    } else if (cmd.startsWith("TILT")) {
      int steps = cmd.substring(4).toInt();
      if (steps >= 0) {
        tiltStepper.step(steps, FORWARD, MICROSTEP);
      } else {
        tiltStepper.step(-steps, BACKWARD, MICROSTEP);
      }
    } else if (cmd == "BEEP") {
      tone(BUZZER_PIN, 1000, 200);
    } else if (cmd == "STOP") {
      panStepper.release();
      tiltStepper.release();
    }
  }
}