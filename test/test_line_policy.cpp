#include "line_policy.h"
#include "test_harness.h"

// The sensor driver and everything that reads a pattern have to agree on the
// bit order. Getting it backwards mirrors the steering.
TEST(pattern_packing_puts_far_left_in_the_high_bit) {
  CHECK_EQ(linePatternPack(true, false, false, false, false), 0x10);
  CHECK_EQ(linePatternPack(false, true, false, false, false), 0x08);
  CHECK_EQ(linePatternPack(false, false, true, false, false), 0x04);
  CHECK_EQ(linePatternPack(false, false, false, true, false), 0x02);
  CHECK_EQ(linePatternPack(false, false, false, false, true), 0x01);
}

TEST(pattern_packing_combines_channels) {
  CHECK_EQ(linePatternPack(false, false, true, true, false), 0x06);
  CHECK_EQ(linePatternPack(true, true, true, true, true), PATTERN_ALL_WHITE);
  CHECK_EQ(linePatternPack(false, false, false, false, false),
           PATTERN_LINE_LOST);
}

// Every pattern has to map to something. An unhandled pattern in the original
// switch fell through with the motors left at their last setting.
TEST(every_pattern_has_a_defined_action) {
  for (uint8_t pattern = 0; pattern < PATTERN_COUNT; ++pattern) {
    const SteerAction action = linePolicyLookup(pattern);
    CHECK(action >= ACTION_CONTEXT && action < ACTION_COUNT);
  }
}

// Only the two boundary patterns should be left to the caller. If a third one
// becomes ACTION_CONTEXT the main loop stops steering without any error.
TEST(only_boundary_patterns_defer_to_the_caller) {
  for (uint8_t pattern = 0; pattern < PATTERN_COUNT; ++pattern) {
    const bool deferred = (linePolicyLookup(pattern) == ACTION_CONTEXT);
    CHECK_EQ(deferred, linePatternIsBoundary(pattern));
  }
}

TEST(centred_on_the_line_drives_straight) {
  const uint8_t centred = linePatternPack(false, false, true, false, false);
  CHECK_EQ(linePolicyLookup(centred), ACTION_FORWARD);
}

// If the line has drifted right of centre the robot is heading left of it and
// has to steer right. A sign error here is the classic line follower bug.
TEST(line_off_to_one_side_steers_towards_it) {
  const uint8_t driftedLeft = linePatternPack(false, false, true, true, false);
  CHECK_EQ(linePolicyLookup(driftedLeft), ACTION_GENTLE_RIGHT);

  const uint8_t driftedRight = linePatternPack(false, true, true, false, false);
  CHECK_EQ(linePolicyLookup(driftedRight), ACTION_GENTLE_LEFT);
}

// The further out the line is, the harder the correction, up to a pivot when
// only an outer channel can still see it.
TEST(correction_strength_increases_with_offset) {
  CHECK_EQ(linePolicyLookup(linePatternPack(false, false, true, true, false)),
           ACTION_GENTLE_RIGHT);
  CHECK_EQ(linePolicyLookup(linePatternPack(false, false, false, true, false)),
           ACTION_MODERATE_RIGHT);
  CHECK_EQ(linePolicyLookup(linePatternPack(false, false, false, false, true)),
           ACTION_PIVOT_RIGHT);

  CHECK_EQ(linePolicyLookup(linePatternPack(false, true, true, false, false)),
           ACTION_GENTLE_LEFT);
  CHECK_EQ(linePolicyLookup(linePatternPack(false, true, false, false, false)),
           ACTION_MODERATE_LEFT);
  CHECK_EQ(linePolicyLookup(linePatternPack(true, false, false, false, false)),
           ACTION_PIVOT_LEFT);
}

TEST(out_of_range_pattern_is_rejected) {
  CHECK_EQ(linePolicyLookup(32), ACTION_CONTEXT);
  CHECK_EQ(linePolicyLookup(255), ACTION_CONTEXT);
}
