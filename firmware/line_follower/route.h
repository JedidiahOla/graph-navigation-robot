// route.h - track layout and junction decisions.
//
// Like line_policy.h this file has no hardware dependency, so the routing can
// be tested on a PC without the robot. See test/.
//
// The track (docs/navigation.md) is a loop with a crossbar through the middle.
// Six numbered junctions, 0 to 5. The crossbar meets each arc at a three-way
// intersection where all five sensors read white without the robot being at a
// numbered junction. Those two are modelled as nodes 6 and 7 so the robot keeps
// track of where it is when it crosses them.
//
// Each node records where the robot ends up if it leaves clockwise,
// anticlockwise, or straight through the middle. Only the two false junctions
// have a straight through exit.

#ifndef ROUTE_H
#define ROUTE_H

#include <stdint.h>

enum {
  NODE_COUNT = 8,
  NODE_WALL = 5,  // dead end, approached with the rangefinder
  NODE_FALSE_A = 6,
  NODE_FALSE_B = 7,
  NODE_NONE = 0xFF
};

// Travel direction around the loop.
typedef enum { HEADING_ACW = 0, HEADING_CW = 1 } Heading;

// Timed turn to run once a junction has been identified.
typedef enum {
  MANOEUVRE_NONE = 0,
  MANOEUVRE_TURN_LEFT_90,
  MANOEUVRE_TURN_RIGHT_90,
  MANOEUVRE_SPIN_180
} Manoeuvre;

typedef struct {
  Manoeuvre manoeuvre;
  uint8_t newHeading;
  // The straight run to the wall at node 5 uses the rangefinder rather than the
  // line sensors, so it is a separate behaviour. Depending on the approach it
  // runs before the turn, after it, or not at all.
  bool approachWallBefore;
  bool approachWallAfter;
} RouteDecision;

// Node the robot moves to when it leaves "current". "destination" is needed
// because the false junctions are only crossed straight through when heading
// for node 1. Returns NODE_NONE if that exit does not exist.
uint8_t routeAdvance(uint8_t current, uint8_t destination, uint8_t heading);

// Works out what to do on arriving at a junction.
//
//   current     node the robot has just arrived at
//   previous    node it came from
//   destination node it is trying to reach
//   heading     current travel direction
//   lastVisited last numbered junction reported to the server
//
// The direction matrix holds either a required heading (0 or 1) or a code for
// junction specific handling (2 to 5). docs/navigation.md lists what each value
// means.
RouteDecision routeDecide(uint8_t current, uint8_t previous,
                          uint8_t destination, uint8_t heading,
                          uint8_t lastVisited);

// Raw matrix lookup. Used by the tests and for debug output.
uint8_t routeMatrix(uint8_t from, uint8_t to);

// True if the node is a real numbered junction rather than a false one.
bool routeIsNumberedJunction(uint8_t node);

#endif  // ROUTE_H
