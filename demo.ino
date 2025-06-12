#include <AFMotor.h>

// Motor & range config
const int PAN_STEPS_PER_REV   = 200;
const int TILT_STEPS_PER_REV  = 200;
const float PAN_RANGE_DEG     = 270.0;
const float TILT_RANGE_DEG    =  60.0;
const int   PAN_RANGE_STEPS   = int((PAN_RANGE_DEG/360.0) * PAN_STEPS_PER_REV);
const int   TILT_HITS         = 3;
const int   TILT_STEP_STEPS   = int(((TILT_RANGE_DEG/TILT_HITS)/360.0) * TILT_STEPS_PER_REV);

// Timing (tweak for speed/smoothness)
const unsigned long PAN_DELAY  = 50;   // ms between each pan micro-step
const unsigned long TILT_DELAY = 300;  // pause after each tilt movement

AF_Stepper panStepper (PAN_STEPS_PER_REV, 1);
AF_Stepper tiltStepper(TILT_STEPS_PER_REV, 2);

void setup() {
  panStepper.setSpeed(30);   // RPM
  tiltStepper.setSpeed(30);
}

void loop() {
  static int  panPos   = 0;
  static int  panDir   = 1;   // 1 = CW, -1 = CCW
  static int  tiltHits = 0;   // how many ups so far in this cycle

  // hit sweep boundary?
  if (abs(panPos) >= PAN_RANGE_STEPS) {
    // flip direction and clamp
    panDir = -panDir;
    panPos = constrain(panPos, -PAN_RANGE_STEPS, PAN_RANGE_STEPS);
    delay(100);

    // tilt logic: up 3×, then reset back down
    if (tiltHits < TILT_HITS) {
      tiltStepper.step(TILT_STEP_STEPS, FORWARD, SINGLE);
      tiltHits++;
    } else {
      tiltStepper.step(TILT_STEP_STEPS * tiltHits, BACKWARD, SINGLE);
      tiltHits = 0;
    }
    delay(TILT_DELAY);
  }

  // one micro-step pan
  if (panDir > 0) {
    panStepper.step(1, FORWARD, SINGLE);
    panPos++;
  } else {
    panStepper.step(1, BACKWARD, SINGLE);
    panPos--;
  }
  delay(PAN_DELAY);
}
