# Original firmware

`Week_5_test_code.ino` is the sketch as originally submitted, kept unmodified so
the version in `firmware/` can be diffed against it.

It does not build without edits: the WiFi credentials and the server address it
originally contained have been removed. Those values are now supplied through
`firmware/line_follower/secrets.h`.

Characteristics of the original worth noting before reading it:

- One file, 865 lines
- The 32-case steering switch appears three times; patterns 1 to 30 are
  identical in all three copies
- Five near-identical moving-average functions differing only in the name of
  their static buffer
- No `pinMode()` calls
- `int timer = millis()`, which overflows
- The HTTP client ignores the result of `client.connect()` and waits a fixed
  1000 ms before reading whatever has arrived
- The routing defect described in `docs/known-issues.md`

A Bluetooth speech-recognition feature was demonstrated alongside the module
project but is not in this file and is not part of this repository. It needed a
second microcontroller acting as a byte pipe and was never integrated into the
mission firmware.
