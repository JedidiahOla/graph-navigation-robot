# Known issues and limitations

Written against the current code. Items marked "original" were problems in the
original firmware. Items marked "remaining" are still there.

## Bug found by the tests (original, fixed)

`routeDecide()` has a fall through branch for the case where no junction specific
turn applies, and the only question is whether the robot is already going the
right way round the loop. The original wrote it like this:

```c
else if (direction ^ NewDir[position1][destination]) {
    TurnACW180();
    direction = NewDir[position1][destination];
}
```

Most cells of the matrix hold a heading, 0 or 1. The rows for the two false
junctions hold codes 2 to 5. Codes 4 and 5 are used by earlier branches, but only
when the robot arrived from node 1. Coming from any other node they reached this
branch, made the XOR non zero, and assigned 4 or 5 into `direction`.

Every later check compares `direction` against 0 or 1. With `direction` set to 4
nothing matched, so the step that advances the position never ran. The robot kept
driving with a position estimate that was stuck at the junction it had left.

This never looked like an obvious failure. It shows up as a wrong turn somewhere
later in a multi waypoint route, which is hard to tell apart from a turn duration
that needs retuning.

It was found by checking every combination of current node, previous node,
destination and heading on a PC, and asserting the resulting heading is still 0
or 1. The fix guards the branch so only real heading values get assigned. The
regression test is `manoeuvre_codes_never_leak_into_the_heading`.

## Position estimate has nothing to check itself against (remaining)

Position comes entirely from counting junction crossings. One missed or double
counted junction puts the robot permanently out of step with the track, and
nothing detects that it has happened. Every decision after that is made on a
wrong position.

Fixing it properly needs a second source of position: encoders and dead reckoning
between junctions, or junction markings that can be told apart. Neither was
available on the module hardware.

## Timed turns drift as the battery discharges (remaining)

`TURN_90_MS` and `TURN_180_MS` are fixed durations at a fixed PWM duty. As the
battery sags the same duty gives less torque, so the same duration gives less
rotation. They were tuned on a full pack, and the robot turns short by the end of
a session. With no encoders there is nothing to correct against.

## Turns and recovery block (remaining)

Turns, the reverse used to re-find the line, and the wall approach all block.
During a 500 ms turn the robot cannot do anything except finish the turn. The
sensor filters keep updating so the readings are current when it ends, but
nothing can interrupt it, and there is no e-stop.

For a robot with one job and no operator this is workable, but it is the first
thing I would change: make every behaviour a tick function so a fault or a stop
can be handled within one control period.

## The fall through branch uses the wrong node (remaining)

That branch looks up `kMatrix[lastReported][destination]`, where `lastReported`
is the last numbered junction sent to the server, not `current`, the node the
robot is standing on. On a false junction these are different. I have left the
behaviour as it was because that is what the robot ran with, but it is probably
not what was intended.

## Sensor faults are not detected (remaining)

A disconnected or dead TCRT5000 reads as a constant value and is treated as a
working channel. The pattern it produces is still a valid table index, so the
robot keeps driving on a steering decision made from a bad reading. Tracking a
health flag per channel and only using the healthy ones would let it degrade
instead of carrying on as if nothing was wrong.

## Not re-tested on hardware (remaining)

The MSP432 and the chassis were lab equipment and I no longer have them. The
logic with no hardware dependency is unit tested, the code is static analysed,
and the rest was transcribed by diffing against the original. It has not been
re-flashed and driven since that work. The behaviour changes are listed below.

## Behaviour changes from the original

| Change | Reason |
| --- | --- |
| Fall through branch guarded to heading values only | Fixes the bug above |
| WiFi gives up after about 12 s instead of looping forever | A typo in the SSID looked the same as a dead access point |
| Arrival POST retries with a doubling delay and reports success or failure | The original assumed every POST worked |
| A failed POST keeps the previous destination | The original used the parsed value without checking it, so a dropped response gave a destination the server never sent |
| Fixed 1000 ms wait replaced with a 2 s read timeout | Faster on a good link, longer on a busy one |
| Rangefinder readings averaged over the sampling window and clamped to 4 to 30 cm | The original sampled for 100 ms but only kept the last reading, and used values outside the range the datasheet specifies |
| `pinMode()` calls added for the motor and sensor pins | They were missing; the pins happened to work in their default state |
| `String` removed from the HTTP code | Heap fragmentation on a target that runs for a long time |
| `int` millis arithmetic changed to `uint32_t` | `int timer = millis()` overflows |
