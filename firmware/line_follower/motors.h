// motors.h - DRV8835 motor driver, PHASE/ENABLE mode.
//
// Everything that writes to a motor pin is in here, so the control logic can be
// read without tracing PWM values and a port to different hardware only has to
// replace this file.

#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"
#include "line_policy.h"

void motorsBegin();

void motorsStop();
void motorsForward();
void motorsReverse();
void motorsCreep();  // forward at CREEP_SPEED, for the wall approach

// Applies a steering action from the policy table. ACTION_CONTEXT is ignored
// because the caller handles the boundary patterns itself.
void motorsApply(SteerAction action);

// Timed turns. These block for TURN_90_MS or TURN_180_MS and keep the sensor
// filters updated while they run, so the array is current when the turn ends.
void motorsSpinLeft90();
void motorsSpinRight90();
void motorsSpin180();

#endif  // MOTORS_H
