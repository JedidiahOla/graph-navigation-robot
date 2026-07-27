// line_policy.h - maps a sensor pattern to a steering action.
//
// This file has no hardware access and no Energia dependency, so it compiles
// into both the firmware and the host test binary. See test/.
//
// The five reflective sensors are packed into a 5 bit pattern, far left in the
// most significant bit:
//
//     bit 4  bit 3  bit 2  bit 1  bit 0
//       FL     L      C      R      FR
//
// A set bit means that channel is over the white line. The pattern is used as
// an index into a 32 entry table of steering actions.
//
// The original sketch had this as a switch statement repeated in three
// functions. Patterns 1 to 30 were the same in all three copies, so they are
// now one table and the caller handles the two that differ.

#ifndef LINE_POLICY_H
#define LINE_POLICY_H

#include <stdint.h>

// Steering actions. Left and right are the direction the robot turns.
typedef enum {
  // Handled by the caller instead of the table. The right response depends on
  // what the robot is currently doing.
  ACTION_CONTEXT = 0,

  ACTION_FORWARD,
  ACTION_GENTLE_LEFT,
  ACTION_MODERATE_LEFT,
  ACTION_SHARP_LEFT,
  ACTION_PIVOT_LEFT,
  ACTION_GENTLE_RIGHT,
  ACTION_MODERATE_RIGHT,
  ACTION_SHARP_RIGHT,
  ACTION_PIVOT_RIGHT,

  ACTION_COUNT
} SteerAction;

// Sensor bit positions.
enum {
  BIT_FAR_RIGHT = 0,
  BIT_RIGHT = 1,
  BIT_CENTRE = 2,
  BIT_LEFT = 3,
  BIT_FAR_LEFT = 4
};

// The two patterns the table cannot resolve on its own.
enum {
  PATTERN_LINE_LOST = 0x00,  // nothing under any sensor
  PATTERN_ALL_WHITE = 0x1F,  // every sensor on white, so a junction
  PATTERN_COUNT = 32
};

// Returns the steering action for a pattern, or ACTION_CONTEXT for the two
// boundary patterns. Out of range input returns ACTION_CONTEXT.
SteerAction linePolicyLookup(uint8_t pattern);

// Packs five channel states into a pattern. Kept here so the sensor driver and
// the tests use the same bit order.
uint8_t linePatternPack(bool farLeft, bool left, bool centre, bool right,
                        bool farRight);

// True if the pattern is one the caller has to handle.
bool linePatternIsBoundary(uint8_t pattern);

#endif  // LINE_POLICY_H
