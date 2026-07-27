# Navigation

## The track

![The track](../media/track.png)

A loop with a crossbar through the middle, a theta shape. Junctions 3 and 2 sit
on the top arc, 4 and 0 on the bottom arc, and 1 on the crossbar. Junction 0 is
the starting position.

Position 5 is not a junction. It is a wall off the left-hand end of the crossbar,
reached by driving straight west from junction 1, past the point where the
crossbar meets the left arc, and off the end of the painted line. The line
sensors are no use for that stretch, so it switches to the rangefinder.

The loop can be driven clockwise or anticlockwise, and which way the robot goes
depends on the junction the server asks for next.

## False junctions

The crossbar meets each arc at a three-way intersection. At both of them all five
sensors read white, exactly as they do on a numbered junction, because the paint
is the same. Ignoring them is not an option: the robot still sees the pattern and
still has to do something about it.

They go into the graph as nodes 6 and 7, each with a straight-through exit that
no real junction has. Node 6 is the right-hand intersection, between junctions 0
and 2. Node 7 is the left-hand one, between 3 and 4. Driving straight through
either puts the robot on the crossbar heading for junction 1.

So the rule is: at an intersection, if the destination is node 1, drive straight
through; otherwise turn off it like any other junction. `routeAdvance()` does
this, and there is a test for it in `test_route.cpp`.

The sensing here is ambiguous: two different situations on the track give the
same reading. Better sensors would not help, because the paint really is the
same in both places. What resolves it is the robot knowing which node it is on,
which comes from having tracked its position since the last junction.

## The track graph

Each node records where the robot ends up leaving clockwise, anticlockwise, or
straight through. `-1` means no such exit.

| Node | CW | ACW | Centre |
| --- | --- | --- | --- |
| 0 | 4 | 6 | - |
| 1 | 6 | 7 | - |
| 2 | 6 | 3 | - |
| 3 | 2 | 7 | - |
| 4 | 7 | 0 | - |
| 5 | 4 | 6 | - |
| 6 (right intersection) | 0 | 2 | 1 |
| 7 (left intersection) | 3 | 4 | 1 |

The rows for 6 and 7 are what identify which intersection is which. Leaving node
6 along an arc reaches 0 or 2, the right-hand pair, so node 6 is the right
intersection. Node 7 reaches 3 or 4, so it is the left one.

The original built this as eight blocks of pointer assignment inside `setup()`.
It is a table, so it is stored as one now, which also lets the tests walk every
edge and check that no exit points off the graph and no node is its own
successor.

## The direction matrix

`kMatrix[from][to]` says what to do on arriving at `from` while heading for
`to`:

| Value | Meaning |
| --- | --- |
| 0 | required heading is anticlockwise |
| 1 | required heading is clockwise |
| 2 | turn to rejoin the loop anticlockwise; direction of turn depends on the node arrived from. Also gates the wall approach. |
| 3 | turn to rejoin the loop clockwise |
| 4 | false-junction case, valid when arriving from node 1 |
| 5 | false-junction case, valid when arriving from node 1 |

Values 2 to 5 appear only in the rows for nodes 6 and 7, which is why they are
false-junction handling. The encoding was derived empirically on the physical
track rather than from first principles, and that shows: the same cell carries
two different kinds of value depending on the row, which caused the bug
described in `known-issues.md`.

## Junction decision flow

```
arrive at junction (pattern 0x1F)
  |
  +-- routeAdvance(current, destination, heading)   update position estimate
  |
  +-- if current == destination:
  |       stop, POST arrival, read next destination
  |       on failure keep the previous destination
  |
  +-- routeDecide(current, previous, destination, heading, lastReported)
  |       -> manoeuvre, new heading, wall-approach flags
  |
  +-- approach wall (if flagged before the turn)  -- ends the mission
  +-- execute manoeuvre: left 90 / right 90 / spin 180 / nothing
  +-- approach wall (if flagged after the turn)   -- ends the mission
  |
  +-- wait until sensors are clear of the junction paint
```

Turns are open-loop and timed: 500 ms for 90 degrees, 600 ms for 180. With no
encoders there is nothing to close the loop against. See `known-issues.md` for
what that costs.
