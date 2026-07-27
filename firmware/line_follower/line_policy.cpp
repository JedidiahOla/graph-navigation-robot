#include "line_policy.h"

// The policy table, taken from the original switch statement.
//
// Pattern bits are FL L C R FR, far left in bit 4. The comment on each row
// lists which channels are over white.
//
// Most of the 32 patterns cannot physically happen: a continuous line cannot
// light the far left and far right channels while leaving the centre dark.
// Those rows are set to ACTION_FORWARD rather than left undefined, so a bad
// reading gives a safe default instead of falling through with the motors at
// their last setting.
//
// clang-format off
static const SteerAction kPolicy[PATTERN_COUNT] = {
    /* 00000  0 */ ACTION_CONTEXT,         // line lost, caller recovers
    /* 00001  1 */ ACTION_PIVOT_RIGHT,     // far right only
    /* 00010  2 */ ACTION_MODERATE_RIGHT,  // right
    /* 00011  3 */ ACTION_PIVOT_RIGHT,     // right, far right
    /* 00100  4 */ ACTION_FORWARD,         // centred
    /* 00101  5 */ ACTION_FORWARD,         // cannot happen
    /* 00110  6 */ ACTION_GENTLE_RIGHT,    // centre, right
    /* 00111  7 */ ACTION_PIVOT_RIGHT,     // centre to far right
    /* 01000  8 */ ACTION_MODERATE_LEFT,   // left
    /* 01001  9 */ ACTION_FORWARD,         // cannot happen
    /* 01010 10 */ ACTION_FORWARD,         // cannot happen
    /* 01011 11 */ ACTION_FORWARD,         // cannot happen
    /* 01100 12 */ ACTION_GENTLE_LEFT,     // left, centre
    /* 01101 13 */ ACTION_FORWARD,         // cannot happen
    /* 01110 14 */ ACTION_FORWARD,         // left to right, wide paint
    /* 01111 15 */ ACTION_SHARP_RIGHT,     // left to far right
    /* 10000 16 */ ACTION_PIVOT_LEFT,      // far left only
    /* 10001 17 */ ACTION_FORWARD,         // cannot happen
    /* 10010 18 */ ACTION_FORWARD,         // cannot happen
    /* 10011 19 */ ACTION_FORWARD,         // cannot happen
    /* 10100 20 */ ACTION_FORWARD,         // cannot happen
    /* 10101 21 */ ACTION_FORWARD,         // cannot happen
    /* 10110 22 */ ACTION_FORWARD,         // cannot happen
    /* 10111 23 */ ACTION_FORWARD,         // cannot happen
    /* 11000 24 */ ACTION_PIVOT_LEFT,      // far left, left
    /* 11001 25 */ ACTION_FORWARD,         // cannot happen
    /* 11010 26 */ ACTION_FORWARD,         // cannot happen
    /* 11011 27 */ ACTION_FORWARD,         // cannot happen
    /* 11100 28 */ ACTION_SHARP_RIGHT,     // far left to centre
    /* 11101 29 */ ACTION_FORWARD,         // cannot happen
    /* 11110 30 */ ACTION_SHARP_LEFT,      // far left to right
    /* 11111 31 */ ACTION_CONTEXT          // junction, caller decides
};
// clang-format on

SteerAction linePolicyLookup(uint8_t pattern) {
  if (pattern >= PATTERN_COUNT) {
    return ACTION_CONTEXT;
  }
  return kPolicy[pattern];
}

uint8_t linePatternPack(bool farLeft, bool left, bool centre, bool right,
                        bool farRight) {
  return (uint8_t)((farLeft ? 1u << BIT_FAR_LEFT : 0u) |
                   (left ? 1u << BIT_LEFT : 0u) |
                   (centre ? 1u << BIT_CENTRE : 0u) |
                   (right ? 1u << BIT_RIGHT : 0u) |
                   (farRight ? 1u << BIT_FAR_RIGHT : 0u));
}

bool linePatternIsBoundary(uint8_t pattern) {
  return pattern == PATTERN_LINE_LOST || pattern == PATTERN_ALL_WHITE;
}
