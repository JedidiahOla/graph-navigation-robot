// config.h - pin assignments and tuning constants.
//
// All the magic numbers from the original sketch are collected here so they
// have names and can be changed in one place. Values marked "tuned" were found
// by trial and error on the track, not calculated.

#ifndef CONFIG_H
#define CONFIG_H

#if defined(ENERGIA)
#include <Energia.h>
#else
#include <Arduino.h>
#endif

// ---------------------------------------------------------------------------
// Motor driver (Pololu DRV8835, PHASE/ENABLE mode)
// ---------------------------------------------------------------------------
// Channel A drives the left motor, channel B the right.
// PHASE sets direction, PWM sets speed.

static const uint8_t PIN_MOTOR_A_PHASE = 36;  // P6.6, green wire
static const uint8_t PIN_MOTOR_B_PHASE = 38;  // P2.4, green wire
static const uint8_t PIN_LEFT_PWM = 39;       // P2.6, yellow wire
static const uint8_t PIN_RIGHT_PWM = 37;      // P5.6, yellow wire

// ---------------------------------------------------------------------------
// TCRT5000 reflective sensor array (five channels across the front)
// ---------------------------------------------------------------------------

static const uint8_t PIN_SENSOR_FAR_LEFT = A6;
static const uint8_t PIN_SENSOR_LEFT = A9;
static const uint8_t PIN_SENSOR_CENTRE = A11;
static const uint8_t PIN_SENSOR_RIGHT = A13;
static const uint8_t PIN_SENSOR_FAR_RIGHT = A14;

// Sharp GP2Y0A41SK0F rangefinder. Only used for the wall at junction 5.
static const uint8_t PIN_DISTANCE = A1;

// ---------------------------------------------------------------------------
// Line detection
// ---------------------------------------------------------------------------
// A TCRT5000 over the white line reflects more IR, which pulls the output low.
// So a reading below the threshold means that channel is on the line.
//
// A fixed threshold only works because the sensor array is enclosed by a light
// shroud (see docs/line-following.md). Without it the value has to be
// recalibrated as the ambient light changes.
static const uint16_t LINE_THRESHOLD = 600;  // tuned, shrouded array

// Samples per channel in the moving average. At roughly 1 kHz this is about a
// 100 ms window: long enough to reject noise, short enough that the controller
// still reacts within one tick.
static const uint8_t SENSOR_FILTER_LENGTH = 100;

// ---------------------------------------------------------------------------
// Motor speeds (PWM duty, 0 to 255)
// ---------------------------------------------------------------------------

static const uint8_t CRUISE_SPEED = 230;  // tuned

// Steering is differential: the inside wheel is slowed by these amounts.
static const uint8_t STEER_DELTA_GENTLE = 25;    // small heading error
static const uint8_t STEER_DELTA_MODERATE = 40;  // larger heading error

// Sharp and pivot turns set absolute speeds rather than a delta, because at
// that point the robot needs to rotate rather than curve.
static const uint8_t SHARP_LEFT_INNER = 50;
static const uint8_t SHARP_LEFT_OUTER = 100;
static const uint8_t SHARP_RIGHT_INNER = 30;
static const uint8_t SHARP_RIGHT_OUTER = 100;
static const uint8_t PIVOT_INNER = 50;
static const uint8_t PIVOT_OUTER = 240;

static const uint8_t REVERSE_SPEED = 50;  // slow, only used to re-find the line
static const uint8_t CREEP_SPEED = 80;    // final approach to the wall
static const uint8_t SPIN_SPEED = 200;    // rotating on the spot, paired with
                                          // the turn durations below

// ---------------------------------------------------------------------------
// Turn durations
// ---------------------------------------------------------------------------
// The chassis has no encoders, so turns are timed rather than measured. These
// were tuned on a full battery and drift as it discharges. See
// docs/known-issues.md.

static const uint16_t TURN_90_MS = 500;
static const uint16_t TURN_180_MS = 600;

// ---------------------------------------------------------------------------
// Control loop periods
// ---------------------------------------------------------------------------
// Sensors are sampled continuously. The controller only acts this often.
// Three different periods, tuned separately for the three behaviours.

static const uint16_t TICK_LINE_FOLLOW_MS = 20;
static const uint16_t TICK_JUNCTION_MS = 30;
static const uint16_t TICK_APPROACH_MS = 50;
static const uint16_t TICK_RECOVER_MS = 40;

// ---------------------------------------------------------------------------
// Wall approach (junction 5)
// ---------------------------------------------------------------------------

static const float OBSTACLE_SLOW_CM = 10.0f;     // drop to CREEP_SPEED
static const float OBSTACLE_STOP_CM = 5.0f;      // start stopping
static const uint16_t STOP_SETTLE_MS = 450;      // coast before braking
static const uint16_t DISTANCE_WINDOW_MS = 100;  // averaging window

// The GP2Y0A41SK0F datasheet specifies 4 cm to 30 cm. Readings outside that
// are not reliable, so they are clamped instead of used.
static const float DISTANCE_MIN_CM = 4.0f;
static const float DISTANCE_MAX_CM = 30.0f;

// Curve fit from the datasheet graph: distance is roughly 13 / V.
static const float DISTANCE_CURVE_K = 13.0f;

// MSP432 ADC: 10 bit result, 5 V reference in this wiring.
static const float ADC_VOLTS_PER_COUNT = 5.0f / 1024.0f;

// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------

static const uint32_t SERIAL_BAUD = 9600;

static const uint8_t WIFI_MAX_ATTEMPTS = 40;  // about 12 s at 300 ms
static const uint16_t WIFI_RETRY_DELAY_MS = 300;

static const uint8_t HTTP_MAX_ATTEMPTS = 3;
static const uint16_t HTTP_RETRY_BASE_MS = 250;  // doubles each attempt
static const uint16_t HTTP_RESPONSE_TIMEOUT_MS = 2000;
static const uint16_t HTTP_BUFFER_SIZE = 512;

#endif  // CONFIG_H
