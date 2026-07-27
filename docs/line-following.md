# Line following

## Sensing

Five TCRT5000 reflective sensors sit on a PCB across the front of the chassis.
Each pairs an IR emitter with a phototransistor behind a visible-light filter.
Over the white line more IR comes back and the output is pulled low, so a
reading **below** `LINE_THRESHOLD` means that channel is on the line.

Each channel is filtered with a 100-sample moving average before it is
thresholded. The raw signal is noisy enough at the line edge that an unfiltered
reading flips between states several times as the sensor crosses it, which the
controller would see as a rapid steering reversal.

The five booleans pack into a 5-bit pattern, far-left in the most significant
bit:

```
bit 4   bit 3   bit 2   bit 1   bit 0
 FL      L       C       R       FR
```

## The threshold problem, and why the fix is cardboard

The threshold is a single fixed number. That only works if the light reaching
the sensors is constant, and it was not: the lab had windows, so the correct
threshold at 10:00 was wrong by 16:00. The first approach was a calibration
routine run at the start of each session, which worked and had to be re-run
every few hours.

The fix that stuck was mechanical. The sensor array was enclosed with a matte
black paper skirt that wraps the sides and comes down to just above the floor,
lifted enough not to drag. Inside the enclosure the illumination is whatever the
emitters provide and nothing else, so the threshold stops moving and one
calibration holds indefinitely.

An adaptive threshold would have kept working, but it is more code and more
state to get wrong, and it still drifts in a room with a moving shadow. The
shroud stops the light changing in the first place, and after fitting it the
threshold was never touched again.

The same reasoning fixed the drive train. The DFRobot motor mounts, fitted as
the manual specifies, let the motors sit loose enough that the wheels cambered
and the chassis jittered when turning on the spot. Mounting the fixings
opposite to the manual's orientation, with tape around the screw mountings,
removed the play. Neither is a clever fix, but both stopped the problem rather
than working around it.

## The policy table

The 32 patterns index a lookup table in `line_policy.cpp` that maps each one to
a steering action. Corrections scale with how far the line has drifted from
centre:

| Pattern | Channels on line | Action | Reasoning |
| --- | --- | --- | --- |
| `00100` | C | forward | centred |
| `00110` | C R | gentle right | line just right of centre |
| `00010` | R | moderate right | line off centre |
| `00001` | FR | pivot right | line about to leave the array |
| `01100` | L C | gentle left | mirror of the above |
| `01000` | L | moderate left | |
| `10000` | FL | pivot left | |
| `01110` | L C R | forward | wide paint, still centred |
| `00000` | none | caller | line lost: reverse to re-acquire |
| `11111` | all | caller | junction, or a false one |

Gentle and moderate corrections slow the inside wheel by 25 and 40 PWM counts
from a cruise of 230. Sharp and pivot actions set absolute speeds instead,
because at that point the robot needs to rotate rather than arc.

### Unreachable patterns

Most of the 32 patterns cannot physically occur. A continuous line cannot light
the far-left and far-right channels while leaving the centre dark, so patterns
like `10001` are not real states. They are populated with `ACTION_FORWARD`
rather than left undefined, so that a spurious reading produces a benign
default instead of dropping the controller into an unhandled case with the
motors still at their last command.

`test_line_policy.cpp` asserts that every one of the 32 entries resolves to a
defined action and that exactly two of them defer to the caller.

## What replaced the original structure

The original sketch had this switch statement three times, once in `loop()`,
once in `FollowLine()` and once in `GoTo5()`. Patterns 1 to 30 were identical in
all three copies; only the two boundary patterns differed, because the right
response to "line lost" or "junction" depends on what the robot is currently
trying to do.

Collapsing them into one table plus two caller-supplied hooks removed about 500
lines and made the policy something that can be read as a table and tested as
one.
