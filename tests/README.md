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

`test_mapping_geometry.py` additionally checks unique-cell overlap against an
independent set oracle, bounded sparse-grid fallback, complete search parity,
cell rotations against continuous replay, preservation of collision weights and
stored alignments, ambiguous-merge isolation, and ICP quality at its returned
pose. These tests use synthetic fixtures and need no private capture files.

The dedicated Home Assistant workflow also installs Home Assistant 2026.8.3 on
Python 3.14 and runs `python tests/ha_smoke.py` in a separate process. That check
imports all integration modules and exercises setup/unload against real HA APIs,
with the robot transport and platform forwarding mocked. It does not start a
real robot or use a Home Assistant token. Correctness lint uses Ruff.

Replay-card checks: `node --check custom_components/openneato/www/openneato-replay-card.js`
and `node --test tests/replay_card.test.cjs`. They cover duplicate registration,
heading interpolation, empty replays and coverage timestamp ordering.

## Firmware recovery transaction

Requires a host C++17 compiler. The harness exercises the actual
`firmware/src/history_recovery.h` with an in-memory filesystem and a small Arduino
String substitute. For example, with Clang or GCC:

```sh
c++ -std=c++17 -Wall -Wextra -I tests/firmware/stubs -I firmware/src tests/firmware/test_history_recovery.cpp -o test_history_recovery
./test_history_recovery
c++ -std=c++17 -Wall -Wextra -I firmware/src tests/firmware/test_frame_recovery.cpp -o test_frame_recovery
./test_frame_recovery
c++ -std=c++17 -Wall -Wextra -I tests/firmware/stubs -I firmware/src tests/firmware/test_checked_json.cpp -o test_checked_json
./test_checked_json
c++ -std=c++17 -Wall -Wextra -I firmware/src tests/firmware/test_compression_output.cpp -o test_compression_output
./test_compression_output
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
