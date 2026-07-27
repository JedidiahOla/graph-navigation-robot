#include "motors.h"

#include "sensors.h"

namespace {

// Sets both PWM duties. Direction is set separately with the PHASE pins.
void drive(uint8_t leftDuty, uint8_t rightDuty) {
  analogWrite(PIN_LEFT_PWM, leftDuty);
  analogWrite(PIN_RIGHT_PWM, rightDuty);
}

void setForwardPhase() {
  digitalWrite(PIN_MOTOR_A_PHASE, LOW);
  digitalWrite(PIN_MOTOR_B_PHASE, LOW);
}

void setReversePhase() {
  digitalWrite(PIN_MOTOR_A_PHASE, HIGH);
  digitalWrite(PIN_MOTOR_B_PHASE, HIGH);
}

// One wheel forward, one back, so the robot rotates about its centre.
void setCounterRotatingPhase() {
  digitalWrite(PIN_MOTOR_A_PHASE, LOW);
  digitalWrite(PIN_MOTOR_B_PHASE, HIGH);
}

// Blocks for durationMs and keeps sampling, so the sensor array reflects the
// surface the robot is over now rather than before the turn started.
void spinFor(uint16_t durationMs) {
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    sensorsUpdate();
  }
}

uint8_t reduced(uint8_t base, uint8_t delta) {
  return (base > delta) ? (uint8_t)(base - delta) : 0;
}

}  // namespace

void motorsBegin() {
  pinMode(PIN_MOTOR_A_PHASE, OUTPUT);
  pinMode(PIN_MOTOR_B_PHASE, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  setForwardPhase();
  motorsStop();
}

void motorsStop() {
  drive(0, 0);
}

void motorsForward() {
  setForwardPhase();
  drive(CRUISE_SPEED, CRUISE_SPEED);
}

void motorsReverse() {
  setReversePhase();
  drive(REVERSE_SPEED, REVERSE_SPEED);
}

void motorsCreep() {
  setForwardPhase();
  drive(CREEP_SPEED, CREEP_SPEED);
}

void motorsApply(SteerAction action) {
  setForwardPhase();

  switch (action) {
    case ACTION_FORWARD:
      drive(CRUISE_SPEED, CRUISE_SPEED);
      break;

    // Turning left means slowing the left wheel.
    case ACTION_GENTLE_LEFT:
      drive(reduced(CRUISE_SPEED, STEER_DELTA_GENTLE), CRUISE_SPEED);
      break;
    case ACTION_MODERATE_LEFT:
      drive(reduced(CRUISE_SPEED, STEER_DELTA_MODERATE), CRUISE_SPEED);
      break;
    case ACTION_SHARP_LEFT:
      drive(SHARP_LEFT_INNER, SHARP_LEFT_OUTER);
      break;
    case ACTION_PIVOT_LEFT:
      drive(PIVOT_INNER, PIVOT_OUTER);
      break;

    case ACTION_GENTLE_RIGHT:
      drive(CRUISE_SPEED, reduced(CRUISE_SPEED, STEER_DELTA_GENTLE));
      break;
    case ACTION_MODERATE_RIGHT:
      drive(CRUISE_SPEED, reduced(CRUISE_SPEED, STEER_DELTA_MODERATE));
      break;
    case ACTION_SHARP_RIGHT:
      drive(SHARP_RIGHT_OUTER, SHARP_RIGHT_INNER);
      break;
    case ACTION_PIVOT_RIGHT:
      drive(PIVOT_OUTER, PIVOT_INNER);
      break;

    case ACTION_CONTEXT:
    default:
      // The caller handles the boundary patterns, so leave the motors alone
      // here rather than overriding whatever it has already set.
      break;
  }
}

// Rotating on one wheel. The stopped wheel is the pivot point.
void motorsSpinLeft90() {
  setForwardPhase();
  drive(0, SPIN_SPEED);
  spinFor(TURN_90_MS);
}

void motorsSpinRight90() {
  setForwardPhase();
  drive(SPIN_SPEED, 0);
  spinFor(TURN_90_MS);
}

// Both wheels driven in opposite directions, so the robot turns about its own
// centre. A full reversal does not fit on the track any other way.
void motorsSpin180() {
  setCounterRotatingPhase();
  drive(SPIN_SPEED, SPIN_SPEED);
  spinFor(TURN_180_MS);
}
