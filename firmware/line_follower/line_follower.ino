// line_follower.ino - autonomous line following robot, DCU EE303.
//
// TI MSP432P401R with a CC3100 WiFi BoosterPack, five TCRT5000 reflective
// sensors and a Sharp GP2Y0A41SK0F rangefinder, on a DFRobot 2WD chassis
// driven through a DRV8835.
//
// The robot follows a white line round a closed track, keeps track of which of
// six numbered junctions it is at, and reports each arrival to a server that
// replies with the next destination. One destination, node 5, is a wall rather
// than a junction. It is approached in a straight line and detected with the
// rangefinder.
//
// Files:
//   line_policy.*  sensor pattern to steering action, no hardware
//   route.*        track layout and junction decisions, no hardware
//   sensors.*      ADC sampling and filtering
//   motors.*       DRV8835 output
//   net.*          WiFi and the waypoint client
//
// The two files with no hardware dependency are unit tested on a PC. See test/.

#include "config.h"
#include "line_policy.h"
#include "motors.h"
#include "net.h"
#include "route.h"
#include "sensors.h"

// ---------------------------------------------------------------------------
// Mission state
// ---------------------------------------------------------------------------

static uint8_t g_currentNode = 0;           // where the robot thinks it is
static uint8_t g_previousNode = NODE_NONE;  // where it came from
static uint8_t g_lastReported = 0;          // last junction reported
static uint8_t g_destination = 0;           // where the server wants it
static uint8_t g_heading = HEADING_ACW;
static bool g_atStart = true;  // stops the first junction advancing position
static uint32_t g_lastTick = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Makes the next control tick happen straight away instead of waiting out the
// rest of the current period. Used after a turn, where the sensor readings have
// changed completely.
void scheduleImmediateTick(uint16_t period) {
  g_lastTick = millis() - period;
}

// True once "period" has passed since the last tick.
bool tickDue(uint16_t period) {
  return (uint32_t)(millis() - g_lastTick) >= period;
}

// Stops and stays stopped. Reaching the wall is the end of the run and there is
// nothing defined to do afterwards.
void haltMission(const char *reason) {
  motorsStop();
  Serial.print("mission complete: ");
  Serial.println(reason);
  for (;;) {
    sensorsUpdate();
  }
}

// ---------------------------------------------------------------------------
// Behaviours
// ---------------------------------------------------------------------------

// All five sensors are over dark surface, so the line has been lost. Reverse
// slowly until one of them finds it again. Reversing works here because the
// only way to lose the line is to drive past it.
void recoverLine() {
  motorsReverse();

  while (sensorsPattern() == PATTERN_LINE_LOST) {
    sensorsUpdate();
    if (tickDue(TICK_RECOVER_MS)) {
      g_lastTick = millis();
    }
  }
}

// Every sensor is over white. After handling a junction the robot has to drive
// clear of the paint before acting on the next reading, or it triggers on the
// same junction again.
void waitUntilClearOfJunction() {
  while (sensorsPattern() == PATTERN_ALL_WHITE) {
    sensorsUpdate();
  }
  g_lastTick = millis();
}

// Straight run to the wall at node 5, using the rangefinder instead of the line
// sensors. Slows at OBSTACLE_SLOW_CM, coasts and stops at OBSTACLE_STOP_CM,
// reports arrival and ends the run.
void approachWall() {
  motorsForward();

  for (;;) {
    const float distance = sensorsDistanceAveragedCm(DISTANCE_WINDOW_MS);

    if (distance < OBSTACLE_STOP_CM) {
      // Coast for a moment so the chassis settles square to the wall instead of
      // stopping dead and rocking forward.
      const uint32_t start = millis();
      while (millis() - start < STOP_SETTLE_MS) {
        sensorsUpdate();
      }
      motorsStop();

      uint8_t next = g_destination;
      netReportArrival(NODE_WALL, &next);
      haltMission("reached the wall at node 5");
    } else if (distance < OBSTACLE_SLOW_CM) {
      motorsCreep();
    }
  }
}

// Follows the line until the robot reaches a junction. Used at startup to get
// off the starting position and onto the track.
void followLineUntilJunction() {
  for (;;) {
    sensorsUpdate();
    if (!tickDue(TICK_LINE_FOLLOW_MS)) {
      continue;
    }
    g_lastTick = millis();

    const uint8_t pattern = sensorsPattern();

    if (pattern == PATTERN_ALL_WHITE) {
      return;
    }
    if (pattern == PATTERN_LINE_LOST) {
      recoverLine();
      scheduleImmediateTick(TICK_LINE_FOLLOW_MS);
      continue;
    }

    motorsApply(linePolicyLookup(pattern));
  }
}

// Arrived at a junction, real or false. Update the position, report it if this
// is the destination, then turn according to the route table.
void handleJunction() {
  if (!g_atStart) {
    const uint8_t next = routeAdvance(g_currentNode, g_destination, g_heading);
    if (next != NODE_NONE) {
      g_previousNode = g_currentNode;
      g_currentNode = next;
    }
  }
  g_atStart = false;

  Serial.print("nav: at node ");
  Serial.println(g_currentNode);

  if (g_destination == g_currentNode) {
    motorsStop();
    g_lastReported = g_destination;

    uint8_t next = g_destination;
    if (netReportArrival(g_lastReported, &next)) {
      g_destination = next;
    } else {
      // Keep the old destination rather than using a failed read. The robot
      // reports again on the next pass.
      Serial.println("nav: destination unchanged after failed report");
    }
  }

  const RouteDecision decision = routeDecide(
      g_currentNode, g_previousNode, g_destination, g_heading, g_lastReported);

  if (decision.approachWallBefore) {
    approachWall();  // does not return
  }

  switch (decision.manoeuvre) {
    case MANOEUVRE_TURN_LEFT_90:
      motorsSpinLeft90();
      break;
    case MANOEUVRE_TURN_RIGHT_90:
      motorsSpinRight90();
      break;
    case MANOEUVRE_SPIN_180:
      motorsSpin180();
      break;
    case MANOEUVRE_NONE:
    default:
      break;
  }
  g_heading = decision.newHeading;

  if (decision.approachWallAfter) {
    approachWall();  // does not return
  }

  waitUntilClearOfJunction();
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  motorsBegin();
  sensorsBegin();

  if (!netBegin()) {
    // No server means no waypoints, and there is no sensible default route to
    // fall back on, so stop here.
    haltMission("no network, cannot get waypoints");
  }

  // Fill the filters before the first control decision, otherwise the first few
  // ticks act on a moving average that is still filling up.
  for (uint8_t i = 0; i < SENSOR_FILTER_LENGTH; ++i) {
    sensorsUpdate();
  }

  g_lastTick = millis();
  followLineUntilJunction();
}

void loop() {
  sensorsUpdate();

  if (!tickDue(TICK_JUNCTION_MS)) {
    return;
  }
  g_lastTick = millis();

  const uint8_t pattern = sensorsPattern();

  if (pattern == PATTERN_LINE_LOST) {
    recoverLine();
    scheduleImmediateTick(TICK_JUNCTION_MS);
    return;
  }

  if (pattern == PATTERN_ALL_WHITE) {
    handleJunction();
    return;
  }

  motorsApply(linePolicyLookup(pattern));
}
