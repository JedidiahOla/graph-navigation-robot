#include "route.h"

// Track layout, taken from the pointer graph the original built by hand in
// setup(). One table instead of eight blocks of pointer assignment, which also
// means the tests can walk every edge. -1 means there is no such exit.
typedef struct {
  int8_t cw;
  int8_t acw;
  int8_t centre;
} RouteNode;

static const RouteNode kNodes[NODE_COUNT] = {
    /* 0 */ {4, 6, -1},
    /* 1 */ {6, 7, -1},
    /* 2 */ {6, 3, -1},
    /* 3 */ {2, 7, -1},
    /* 4 */ {7, 0, -1},
    /* 5 */ {4, 6, -1},
    /* 6 */ {0, 2, 1},  // crossbar meets the right arc, through leads to 1
    /* 7 */ {3, 4, 1}   // crossbar meets the left arc, through leads to 1
};

// kMatrix[from][to] says what to do at "from" when heading for "to". Values 0
// and 1 are headings (anticlockwise, clockwise). Values 2 to 5 select junction
// specific handling in routeDecide().
static const uint8_t kMatrix[NODE_COUNT][NODE_COUNT] = {
    {0, 0, 0, 0, 1, 0, 0, 1}, {1, 1, 1, 0, 0, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 1}, {1, 0, 1, 1, 0, 0, 1, 0},
    {0, 1, 0, 1, 1, 1, 1, 0}, {1, 1, 1, 1, 1, 1, 1, 1},
    {4, 2, 4, 4, 1, 2, 2, 1}, {0, 3, 1, 5, 5, 2, 2, 1}};

uint8_t routeMatrix(uint8_t from, uint8_t to) {
  if (from >= NODE_COUNT || to >= NODE_COUNT) {
    return 0;
  }
  return kMatrix[from][to];
}

bool routeIsNumberedJunction(uint8_t node) {
  return node < NODE_FALSE_A;
}

uint8_t routeAdvance(uint8_t current, uint8_t destination, uint8_t heading) {
  if (current >= NODE_COUNT) {
    return NODE_NONE;
  }

  // Crossing a false junction straight through is only correct when node 1 is
  // the target. Otherwise the robot turns off it like any other junction.
  const bool onFalseJunction =
      (current == NODE_FALSE_A || current == NODE_FALSE_B);
  if (onFalseJunction && destination == 1) {
    return (uint8_t)kNodes[current].centre;
  }

  const int8_t next =
      (heading == HEADING_ACW) ? kNodes[current].acw : kNodes[current].cw;
  return (next < 0) ? (uint8_t)NODE_NONE : (uint8_t)next;
}

RouteDecision routeDecide(uint8_t current, uint8_t previous,
                          uint8_t destination, uint8_t heading,
                          uint8_t lastVisited) {
  RouteDecision decision;
  decision.manoeuvre = MANOEUVRE_NONE;
  decision.newHeading = heading;
  decision.approachWallBefore = false;
  decision.approachWallAfter = false;

  if (current >= NODE_COUNT || destination >= NODE_COUNT) {
    return decision;
  }

  const uint8_t code = kMatrix[current][destination];

  if (code == 2) {
    // Arriving from node 1 with the wall as the target means the straight run
    // happens first, then the turn rejoins the loop.
    if (previous == 1 && destination == NODE_WALL) {
      decision.approachWallBefore = true;
    }
    decision.manoeuvre = (previous == 0 || previous == 4)
                             ? MANOEUVRE_TURN_LEFT_90
                             : MANOEUVRE_TURN_RIGHT_90;
    decision.newHeading = HEADING_ACW;
    if (destination == NODE_WALL) {
      decision.approachWallAfter = true;
    }
  } else if (code == 3) {
    decision.manoeuvre =
        (previous == 3) ? MANOEUVRE_TURN_LEFT_90 : MANOEUVRE_TURN_RIGHT_90;
    decision.newHeading = HEADING_CW;
  } else if (code == 4 && previous == 1) {
    const bool right = (destination == 0);
    decision.manoeuvre =
        right ? MANOEUVRE_TURN_RIGHT_90 : MANOEUVRE_TURN_LEFT_90;
    decision.newHeading = right ? HEADING_CW : HEADING_ACW;
  } else if (code == 5 && previous == 1) {
    const bool right = (destination == 3);
    decision.manoeuvre =
        right ? MANOEUVRE_TURN_RIGHT_90 : MANOEUVRE_TURN_LEFT_90;
    decision.newHeading = right ? HEADING_CW : HEADING_ACW;
  } else if (lastVisited < NODE_COUNT) {
    // Nothing junction specific applies, so the only question is whether the
    // robot is already going the right way round the loop.
    const uint8_t required = kMatrix[lastVisited][destination];

    // Bug fix. Most cells of the matrix hold a heading, 0 or 1. The rows for
    // the two false junctions hold codes 2 to 5. Codes 4 and 5 are only used by
    // the branches above when the robot arrived from node 1. Coming from any
    // other node they reached this branch, and the original assigned the matrix
    // value straight into the heading variable:
    //
    //     else if (direction ^ NewDir[position1][destination]) {
    //         TurnACW180();
    //         direction = NewDir[position1][destination];
    //     }
    //
    // Every later check compares that variable against 0 or 1, so a value of 4
    // or 5 matched nothing. The robot stopped updating its position and drove
    // the rest of the route thinking it was still at the junction it had left.
    //
    // Found by the host tests rather than on the track. See
    // docs/known-issues.md.
    if (required <= HEADING_CW && (heading ^ required)) {
      decision.manoeuvre = MANOEUVRE_SPIN_180;
      decision.newHeading = required;
    }
  }

  return decision;
}
