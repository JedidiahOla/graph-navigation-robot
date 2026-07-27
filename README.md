# Graph Navigation Robot

[![ci](https://github.com/YOUR_USERNAME/graph-navigation-robot/actions/workflows/ci.yml/badge.svg)](https://github.com/YOUR_USERNAME/graph-navigation-robot/actions/workflows/ci.yml)

An autonomous robot that models the track it is driving on as a graph, and uses
that graph to reach junctions requested by a server over WiFi.

It can only see the 5 cm of white line under its nose. Five reflective IR sensors
pack into a 5-bit pattern that indexes a 32-entry steering table, which keeps it
on the line. Everything above that is graph traversal: the robot counts junction
crossings to track which node it is on, looks up the turn that gets it closer to
its destination, and reports each arrival over HTTP to get the next one. Two
points on the track give a junction reading without being junctions, and those
are resolved from the graph rather than from the sensors.

Mobile Robotics module project, Dublin City University. Three-person group
project: I wrote the firmware, the other two built and wired the hardware.
C++/Energia on an MSP432, 19 tests.

![The track](media/track.png)

*The track, as the waypoint server draws it. Six numbered junctions, and a wall
at 5.*

## The problem

The track is a loop with a crossbar through the middle, a theta shape. The top
arc carries junctions 3 and 2, the bottom arc carries 4 and 0, and the crossbar
carries junction 1. Position 5 is not a junction at all: it is a wall off the
left-hand end of the crossbar, reached by driving straight west past where the
crossbar meets the left arc and off the end of the painted line.

The server sends a list of junctions and the robot has to visit them in order. It
has no encoders, no IMU and no odometry, so it cannot measure how far it has
driven or how far it has turned.

Three constraints drive most of the design. No position feedback means the robot
has to infer where it is by counting junctions as it crosses them, so the routing
table has to be right first time because there is nothing to correct it against.
No rotation feedback means turns are open loop and timed, tuned on a full battery
and drifting as it discharges.

And the sensing is ambiguous. The crossbar meets each arc at a three-way
intersection, and at both of those all five sensors read white exactly as they do
on a real junction. Better sensors don't help, because the paint really is
identical. The only thing separating the two cases is the robot already knowing
which node it is standing on.

## How it fits together

```
SENSORS                        CONTROL                     ACTUATION
+----------------------+  +----------------------+  +--------------------+
| 5x TCRT5000          |  | every pass:          |  | DRV8835            |
|  analogRead          +->+  sample + filter     |  |  PHASE/ENABLE      |
|  100-sample average  |  |                      |  |                    |
|  threshold -> 5 bits |  | every 30 ms:         +->+  left PWM          |
+----------------------+  |  pattern -> action   |  |  right PWM         |
                          |                      |  +--------------------+
+----------------------+  |  0x00 -> recover     |
| GP2Y0A41SK0F         +->+  0x1F -> junction    |
|  wall approach only  |  |  else -> steer       |
+----------------------+  +----------+-----------+
                                     |
                                     v
                          +----------------------+
                          | route graph          |
                          |  8 nodes, 6 real     |
                          |  2 false junctions   |
                          |  8x8 direction matrix|
                          +----------+-----------+
                                     |
                                     v
                          +----------------------+
                          | CC3100 WiFi          |
                          |  POST position=N     |
                          |  <- next destination |
                          +----------------------+
```

Steering and routing are pure functions with no hardware access, so both compile
and run on a PC. Everything that writes to a pin sits behind three driver files.

## Steering

Each TCRT5000 pairs an IR emitter with a phototransistor. Over white more IR
comes back and the output is pulled low, so a reading below the threshold means
that sensor is on the line. Each channel goes through a 100-sample moving average
first, which is about a 100 ms window. Raw, the signal flips state several times
as a sensor crosses the line edge, and the controller reads that as a steering
reversal.

The five results pack into a pattern, far left in the top bit, and index a table:

| Pattern | On the line | Action | Left / right PWM |
| --- | --- | --- | --- |
| `00100` | C | straight | 230 / 230 |
| `00110` | C R | gentle right | 230 / 205 |
| `00010` | R | moderate right | 230 / 190 |
| `11100` | FL L C | sharp right | 100 / 30 |
| `00001` | FR | pivot right | 240 / 50 |
| `00000` | none | recover | reverse until reacquired |
| `11111` | all | junction | see below |

Corrections scale with offset. Gentle and moderate slow the inside wheel by a
fixed amount so the robot arcs back. Sharp and pivot set absolute speeds instead,
because by then the line is nearly out of the array and it needs to rotate.

Most of the 32 patterns can't physically occur, since a continuous line can't
light the outer sensors while leaving the centre dark. Those entries hold
"straight" rather than being left undefined, so a bad reading gives a safe
default instead of falling through with the motors at their last setting.

## The false junction problem

All five sensors white means a junction. It also means one of the two three-way
intersections where the crossbar meets an arc, which look identical from the
sensors and are not numbered positions.

They're handled by putting them in the track graph as nodes 6 and 7. Every node
records where the robot ends up leaving clockwise or anticlockwise, and the two
intersections also have a straight-through exit that no real junction has:

```
Node   CW   ACW   Through
 0      4    6      -
 1      6    7      -
 2      6    3      -
 3      2    7      -
 4      7    0      -
 5      4    6      -
 6      0    2      1      <- crossbar meets the right arc
 7      3    4      1      <- crossbar meets the left arc
```

Node 6 sits where the crossbar meets the right arc, so leaving it along the arc
reaches 0 one way and 2 the other. Node 7 sits at the left end, between 3 and 4.
Driving straight through either one puts the robot on the crossbar heading for
junction 1, which is why both have the same through exit.

So heading for node 1, the robot crosses an intersection straight through.
Heading anywhere else, it turns off it like a normal junction. The ambiguous
reading is resolved from state the robot has been carrying since the last
junction, not from the sensors.

An 8x8 matrix then says what to do at each node for each destination. Most cells
hold a required heading, 0 or 1. The rows for the two intersections hold codes 2
to 5, which select junction-specific handling. `routeDecide()` takes the current
node, previous node, destination, heading and last reported junction, and returns
a turn plus the new heading. That signature is why the routing is testable: there
is no hardware in it, so every combination can be enumerated on a laptop. Doing
that turned up a real bug, written up in [docs/known-issues.md](docs/known-issues.md).

Turns are timed: 500 ms for 90 degrees, driving one wheel so the stopped one is
the pivot, and 600 ms for 180 degrees, driving both in opposite directions so the
robot turns about its own centre. That's the only way a reversal fits on the
track.

## Repository layout

```
firmware/
  line_follower/      the firmware; open line_follower.ino in Energia
    line_follower.ino   mission state and top-level behaviours
    line_policy.*       sensor pattern -> steering action (no hardware)
    route.*             track graph and junction decisions (no hardware)
    sensors.*           ADC sampling and filtering
    motors.*            DRV8835 output
    net.*               CC3100 WiFi and the waypoint client
    config.h            pin map and tuning constants

test/                 host-native tests for the two hardware-free modules
docs/                 architecture, line following, navigation, known issues
legacy/               the original single-file sketch, for comparison
```

Longer write-ups are in [docs/architecture.md](docs/architecture.md),
[docs/line-following.md](docs/line-following.md) and
[docs/navigation.md](docs/navigation.md).

## Quick start

The steering and routing tests run on their own, no hardware needed:

```
cmake -S test -B build
cmake --build build
ctest --test-dir build --output-on-failure   # 19 tests
```

They cover the sensor bit packing, every entry of the steering table, every edge
of the track graph, and every combination of inputs to `routeDecide()`.

For the firmware, use the standard Arduino IDE. Energia, the TI fork of Arduino
this was written in, is no longer maintained and its download host is offline,
but the MSP432 core still works under Arduino. Add this to `Additional Boards
Manager URLs` in Arduino preferences:

```
https://raw.githubusercontent.com/Andy4495/TI_Platform_Cores_For_Arduino/main/json/package_energia_optimized_index.json
```

Then `Tools -> Board -> Boards Manager`, search for MSP432 and install the
platform. That repository mirrors the board packages itself, so it does not
depend on the dead Energia download host.

```
cp firmware/line_follower/secrets.h.example firmware/line_follower/secrets.h
# set WIFI_SSID, WIFI_PASSWORD, SERVER_HOST, SERVER_PORT, SERVER_ARRIVAL_PATH
```

Open `firmware/line_follower/line_follower.ino`, select MSP-EXP432P401R and
upload. Serial monitor at 9600 baud for position and network logging. `secrets.h`
is gitignored; the original sketch had the credentials in the main source file,
which is how they end up in a public repo.

The CC3100 WiFi library shipped with the Energia application rather than with the
board core, so the Boards Manager does not install it. It is in the
[Energia repository](https://github.com/energia/Energia/tree/master/libraries)
and can be copied into the Arduino `libraries` folder.

## Status

**Bench and track tested on the course hardware. It completed every weekly
objective, but did not fully demonstrate on assessment day.** The firmware here
is the same behaviour reorganised into modules, with tests and one bug fix added.
The MSP432 and the chassis were lab equipment I no longer have access to, so it
hasn't been re-flashed since that work.

What isn't solid:

- **The position estimate has nothing to check itself against.** One missed or
double-counted junction puts the robot permanently out of step with the track and
nothing detects it. Every decision after that is made on a wrong position.
- **Turns drift as the battery discharges.** Fixed durations at a fixed duty. The
same 500 ms gives less rotation on a sagging pack, so the robot turns short by
the end of a session.
- **Turns and line recovery block.** During a 500 ms turn nothing else can
happen. There's no e-stop. Making every behaviour a tick function is the first
thing I'd change.
- **Sensor faults aren't detected.** A dead TCRT5000 reads as a constant and is
treated as a working channel, so the robot keeps steering on a reading it should
be ignoring.
- **The assessment day failure was partly the client.** The original ignored the
return of `client.connect()`, waited a fixed 1000 ms, and used the parsed
response without checking it. It had only ever run on a network that answered.
The client here retries three times with a doubling delay, uses a 2 s read
timeout, validates the response, and keeps the previous destination if all three
attempts fail.

The routing and the steering policy are the parts I'd defend. The open-loop
motion is the part that needs hardware it didn't have.

## Hardware

TI MSP432P401R (ARM Cortex-M4F, 256 KB flash, 64 KB SRAM), TI CC3100 WiFi
BoosterPack over SPI, five Vishay TCRT5000 reflective sensors on a front PCB, a
Sharp GP2Y0A41SK0F IR rangefinder, a Pololu DRV8835 dual motor driver in
PHASE/ENABLE mode, and a DFRobot 170 mm 2WD chassis with a rear caster. The pin
map is in [config.h](firmware/line_follower/config.h).

Two fixes on the hardware side mattered more than any code I wrote. The motor
mounts, fitted the way the manual shows, left the wheels cambered enough to
jitter when turning on the spot; mounting the fixings the opposite way round
removed the play. And the fixed light threshold kept drifting because the lab had
windows, so the sensor array was enclosed in a matte black skirt coming down to
just above the floor. Inside the shroud the only light is what the emitters
produce, and after that the threshold was never touched again, which is what
makes a single hardcoded `LINE_THRESHOLD` viable at all.

<!-- Add a photo of the sensor array and shroud here if one survives:
![Sensor array and shroud](media/sensor-array.jpg)
*Front sensor PCB on non-metal spacers, raised clear of the metal grill after
some solder joints turned out to be long enough to touch it.* -->

## References

- Texas Instruments, *MSP432P401R SimpleLink Microcontroller* datasheet
- Texas Instruments, *CC3100 SimpleLink Wi-Fi User Guide*, SWRU371B
- Vishay, *TCRT5000 Reflective Optical Sensor* datasheet
- Sharp, *GP2Y0A41SK0F Distance Measuring Sensor* datasheet, via Pololu
- Pololu, *DRV8835 Dual Motor Driver Carrier* documentation
- DFRobot, *Turtle 2WD Mobile Platform* instruction manual

## License

MIT. See [LICENSE](LICENSE).
