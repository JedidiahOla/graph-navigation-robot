# Architecture

## Constraints the design follows from

The robot has no encoders, no IMU, and no odometry of any kind. The only inputs
are five reflective sensors reading a binary surface, one IR rangefinder, and
whatever the waypoint server says. Everything below is shaped by that: the
robot cannot measure how far it has travelled or how far it has turned, so
position is inferred from junction crossings and rotation is open-loop and
timed.

## Module layout

```
                        +---------------------------+
                        |     line_follower.ino     |
                        |  mission state, behaviours|
                        +---------------------------+
                          |        |        |      |
        +-----------------+        |        |      +---------------+
        |                          |        |                      |
+---------------+     +---------------+  +---------------+  +---------------+
|  line_policy  |     |     route     |  |    sensors    |  |    motors     |
| pattern ->    |     | graph +       |  | ADC + filters |  | DRV8835 PWM   |
| steer action  |     | junction logic|  |               |  |               |
+---------------+     +---------------+  +---------------+  +---------------+
        \                     /                  |                  |
         \                   /                   +--------+---------+
          no hardware deps: unit tested                   |
          on the host, see test/                    +---------------+
                                                    |      net      |
                                                    | CC3100 + HTTP |
                                                    +---------------+
```

The split is between code that only works on data and code that has to touch a
pin. `line_policy` and `route` are the first kind, so they compile into a host
binary as they are and the unit tests cover them. The firmware and the tests
build the same two files rather than copies.

## Control loop

Sensors are sampled on every pass of `loop()`; the controller acts on a slower
fixed cadence, so the moving-average filters see far more samples than there are
control decisions.

```
loop()
 |
 +-- sensorsUpdate()                 every pass, ~1 kHz
 |
 +-- if not tickDue(30 ms): return
 |
 +-- pattern = sensorsPattern()      five channels -> 5 bits
     |
     +-- 0x00  line lost      -> recoverLine()
     +-- 0x1F  all white      -> handleJunction()
     +-- else                 -> motorsApply(linePolicyLookup(pattern))
```

Three different tick periods are in use, inherited from the original tuning:
20 ms while following a line to a junction, 30 ms in the main mission loop,
50 ms during the wall approach. They were not derived; they were adjusted until
the robot stopped oscillating.

## Junction handling

`0x1F` (every sensor over white) means the robot is on a painted junction, or
on one of the two unpainted points where the track crosses itself and produces
the same reading. `handleJunction()`:

1. Advances the position estimate along the track graph.
2. If this is the destination, reports arrival and reads back the next one.
3. Asks `routeDecide()` what manoeuvre the new destination requires.
4. Executes the manoeuvre, then waits until the sensors are clear of the
   junction so the next tick does not re-trigger on the same paint.

`routeDecide()` only depends on its arguments, so every combination of them can
be checked on a PC. That is how the bug in `known-issues.md` was found.

## Wall approach

Node 5 is a wall, not a junction. The straight run to it is a separate behaviour
that ignores the line sensors and drives on the rangefinder: cruise, drop to
`CREEP_SPEED` at 10 cm, coast for `STOP_SETTLE_MS` and brake at 5 cm, report
arrival, halt. It is entered only from `routeDecide()` and never returns; the
mission ends at the wall.

## Failure behaviour

| Failure | Response |
| --- | --- |
| WiFi does not associate at boot | Halt. Without waypoints there is no route to drive. |
| Arrival POST fails | Retry three times with exponential backoff, then keep the previous destination rather than acting on a garbage value. |
| Response malformed or out of range | Treated as a failed POST. |
| Line lost mid-track | Reverse slowly until any channel re-acquires it. |
| Rangefinder reads outside 4-30 cm | Clamped to the specified range rather than acted on. |
