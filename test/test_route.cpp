#include "route.h"
#include "test_harness.h"

// The graph was typed in by hand from the track. A mistake in it is hard to see
// by reading and shows up as the robot driving to the wrong junction, so check
// the structure instead.
TEST(every_exit_leads_to_a_real_node) {
  for (uint8_t node = 0; node < NODE_COUNT; ++node) {
    for (uint8_t destination = 0; destination < NODE_COUNT; ++destination) {
      const uint8_t cw = routeAdvance(node, destination, HEADING_CW);
      const uint8_t acw = routeAdvance(node, destination, HEADING_ACW);
      CHECK(cw < NODE_COUNT);
      CHECK(acw < NODE_COUNT);
    }
  }
}

TEST(no_node_is_its_own_successor) {
  for (uint8_t node = 0; node < NODE_COUNT; ++node) {
    // Destination 0 avoids the false-junction straight-through case.
    CHECK(routeAdvance(node, 0, HEADING_CW) != node);
    CHECK(routeAdvance(node, 0, HEADING_ACW) != node);
  }
}

TEST(unknown_node_is_rejected) {
  CHECK_EQ(routeAdvance(NODE_COUNT, 0, HEADING_CW), NODE_NONE);
  CHECK_EQ(routeAdvance(200, 0, HEADING_ACW), NODE_NONE);
}

// The two false junctions light all five sensors but are not numbered
// positions. Heading for node 1 the robot drives straight through them.
// Otherwise it turns off them like any other junction.
TEST(false_junctions_are_crossed_straight_through_only_towards_node_1) {
  CHECK_EQ(routeAdvance(NODE_FALSE_A, 1, HEADING_CW), 1);
  CHECK_EQ(routeAdvance(NODE_FALSE_B, 1, HEADING_ACW), 1);

  CHECK(routeAdvance(NODE_FALSE_A, 3, HEADING_CW) != 1);
  CHECK(routeAdvance(NODE_FALSE_B, 3, HEADING_ACW) != 1);
}

TEST(numbered_junctions_are_distinguished_from_false_ones) {
  for (uint8_t node = 0; node <= 5; ++node) {
    CHECK(routeIsNumberedJunction(node));
  }
  CHECK(!routeIsNumberedJunction(NODE_FALSE_A));
  CHECK(!routeIsNumberedJunction(NODE_FALSE_B));
}

// A decision has to leave the heading valid whatever it was given.
TEST(decisions_always_produce_a_valid_heading) {
  for (uint8_t current = 0; current < NODE_COUNT; ++current) {
    for (uint8_t destination = 0; destination < NODE_COUNT; ++destination) {
      for (uint8_t previous = 0; previous < NODE_COUNT; ++previous) {
        for (uint8_t heading = 0; heading <= 1; ++heading) {
          const RouteDecision d =
              routeDecide(current, previous, destination, heading, previous);
          CHECK(d.newHeading == HEADING_CW || d.newHeading == HEADING_ACW);
          CHECK(d.manoeuvre >= MANOEUVRE_NONE &&
                d.manoeuvre <= MANOEUVRE_SPIN_180);
        }
      }
    }
  }
}

// The wall run should only start when node 5 is the destination. Starting it
// anywhere else drives the robot off the track in a straight line.
TEST(wall_approach_only_triggers_for_node_5) {
  for (uint8_t current = 0; current < NODE_COUNT; ++current) {
    for (uint8_t destination = 0; destination < NODE_COUNT; ++destination) {
      if (destination == NODE_WALL) {
        continue;
      }
      for (uint8_t previous = 0; previous < NODE_COUNT; ++previous) {
        const RouteDecision d =
            routeDecide(current, previous, destination, HEADING_ACW, previous);
        CHECK(!d.approachWallBefore);
        CHECK(!d.approachWallAfter);
      }
    }
  }
}

// Approaching the wall from node 1 is the case the demonstration needed:
// straight run first, then rejoin the loop.
TEST(wall_approach_from_node_1_runs_before_the_turn) {
  const uint8_t current = 6;  // matrix entry 2, reachable heading for node 5
  if (routeMatrix(current, NODE_WALL) == 2) {
    const RouteDecision d = routeDecide(current, 1, NODE_WALL, HEADING_ACW, 1);
    CHECK(d.approachWallBefore);
  }
}

TEST(out_of_range_decision_is_a_no_op) {
  const RouteDecision d = routeDecide(NODE_COUNT, 0, 0, HEADING_CW, 0);
  CHECK_EQ(d.manoeuvre, MANOEUVRE_NONE);
  CHECK_EQ(d.newHeading, HEADING_CW);
  CHECK(!d.approachWallBefore);
  CHECK(!d.approachWallAfter);
}

TEST(matrix_lookup_is_bounds_checked) {
  CHECK_EQ(routeMatrix(NODE_COUNT, 0), 0);
  CHECK_EQ(routeMatrix(0, NODE_COUNT), 0);
}

// Regression test for the bug fixed in route.cpp. Covers the exact case that
// caused it: a matrix cell holding a manoeuvre code instead of a heading,
// reached from a node other than 1 so the earlier branches do not catch it.
TEST(manoeuvre_codes_never_leak_into_the_heading) {
  for (uint8_t lastVisited = 0; lastVisited < NODE_COUNT; ++lastVisited) {
    for (uint8_t destination = 0; destination < NODE_COUNT; ++destination) {
      if (routeMatrix(lastVisited, destination) <= HEADING_CW) {
        continue;  // a real heading, not the case being tested
      }
      for (uint8_t previous = 0; previous < NODE_COUNT; ++previous) {
        if (previous == 1) {
          continue;  // consumed by the code 4 / code 5 branches
        }
        const RouteDecision d = routeDecide(lastVisited, previous, destination,
                                            HEADING_ACW, lastVisited);
        CHECK(d.newHeading == HEADING_ACW || d.newHeading == HEADING_CW);
      }
    }
  }
}
