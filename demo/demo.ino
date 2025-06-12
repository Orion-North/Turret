#include <AFMotor.h>

// hardware & geometry
const int   STEPS_PER_REV     = 200;    // full-step motor
const float PAN_GEAR_RATIO    = 6.0;    // 1:6 reduction on pan
const float PAN_SWEEP_DEG     = 160.0;  // turret sweep at output
const float TILT_SWEEP_DEG    =  60.0;  // total tilt range
const int   TILT_STEPS_COUNT  =   3;    // ups before reset

// motor speeds (RPM)
const uint16_t PAN_RPM  = 200;  // slower pan for extra smoothness
const uint16_t TILT_RPM = 30;  // tilt can be snappier

// computed step counts
const int PAN_STEPS = int(STEPS_PER_REV * PAN_GEAR_RATIO * PAN_SWEEP_DEG / 360.0);
const int TILT_STEP = int(STEPS_PER_REV * (TILT_SWEEP_DEG / TILT_STEPS_COUNT) / 360.0);

// instantiate steppers
AF_Stepper panStepper (STEPS_PER_REV, 1);
AF_Stepper tiltStepper(STEPS_PER_REV, 2);

void setup() {
  panStepper.setSpeed(PAN_RPM);
  tiltStepper.setSpeed(TILT_RPM);
}

void loop() {
  static int tiltCount = 0;

  // 1) sweep pan forward
  panStepper.step(PAN_STEPS, FORWARD, INTERLEAVE);

  // tilt at end
  if (tiltCount < TILT_STEPS_COUNT) {
    tiltStepper.step(TILT_STEP, FORWARD, INTERLEAVE);
    tiltCount++;
  } else {
    tiltStepper.step(TILT_STEP * tiltCount, BACKWARD, INTERLEAVE);
    tiltCount = 0;
  }

  // 2) sweep pan backward
  panStepper.step(PAN_STEPS, BACKWARD, INTERLEAVE);

  // tilt at end again
  if (tiltCount < TILT_STEPS_COUNT) {
    tiltStepper.step(TILT_STEP, FORWARD, INTERLEAVE);
    tiltCount++;
  } else {
    tiltStepper.step(TILT_STEP * tiltCount, BACKWARD, INTERLEAVE);
    tiltCount = 0;
  }
}
