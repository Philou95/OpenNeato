# Offline regression tests

Run from the repository root. These tests never contact Home Assistant or the robot.

## Python history downloads and scan delivery

Requires Python 3.12+, `aiohttp`, and `Pillow`. Home Assistant services and the
network transport are simulated; the integration's API and runner code are imported.

```sh
python -m unittest discover -s tests -p 'test_*.py' -v
```

Covers concurrent downloads, partial lines and offset fallback, response size
limits, reboot and duplicate scan handling, saved cursors, and overlapping ticks.
This does not replace testing inside a running Home Assistant installation.

## Firmware recovery transaction

Requires a host C++17 compiler. The harness exercises the actual
`firmware/src/history_recovery.h` with an in-memory filesystem and a small Arduino
String substitute. For example, with Clang or GCC:

```sh
c++ -std=c++17 -Wall -Wextra -I tests/firmware/stubs -I firmware/src tests/firmware/test_history_recovery.cpp -o test_history_recovery
./test_history_recovery
```

On Windows, use `zig c++` in place of `c++`, output `test_history_recovery.exe`,
and execute `.\test_history_recovery.exe`. Do not define `NDEBUG`: assertions
are the harness's checks.

Simulates interruption after every mutating filesystem operation, short writes,
failed renames, corrupt output, and unreadable sources. It verifies preservation
and exactly-once merging after retry. Physical power loss inside a SPIFFS
operation and actual flash wear are outside this simulation.

## Firmware and frontend validation

```sh
pio run -e c3-debug
python scripts/check_format.py
pio check -e c3-debug --fail-on-defect=low
```

Run `npm run build` in `frontend/` to check formatting, lint, duplication,
OpenAPI types and firmware route parity, then build the embedded assets.
