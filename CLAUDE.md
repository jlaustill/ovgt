# OVGT — Project Notes for Claude

Variable-geometry turbo controller. Teensy 4.1 / PlatformIO / Arduino, migrating
C++ → C-Next. (C-Next syntax lives in the global ~/.claude/CLAUDE.md.)

## 🌟 North Star (read first for any CE / sensor / control work)
The objective of this whole project is to **control the VGT vanes to maximize
compressor efficiency (CE) at all times.** CE is the project's objective function;
making it *accurate enough to control on* (precision **and** time-alignment, which
trade off — lag dominates) is the critical path. **Read `docs/north-star.md` before
planning anything CE-, sensor-, or control-related** and judge each step by: *does
this make CE more accurate, or let us act on it?* Phased plan:
`docs/plans/2026-06-22-realtime-ce-accuracy-epic.md`.

## Git workflow
- Solo private repo: commit directly to `main`, **no feature branches**, frequent
  small commits. Push only when asked.

## Firmware (Teensy 4.1)
- Build: `pio run -e teensy41` · Flash: `pio run -e teensy41 -t upload`
- Teensy = `/dev/ttyACM0`, 115200 baud, USB VID:PID `16c0:0483`.
- Capture serial WITHOUT resetting the board: `stty -F /dev/ttyACM0 115200 raw -echo`
  then `timeout <s> cat /dev/ttyACM0` (Teensy USB CDC does not reset on port-open).

## Logic tests (host-side, no hardware)
- `pio test -e native` (Unity); filter with `-f test_<name>`.
- A newly tested `src/` file must be added to `[env:native]` `build_src_filter` in `platformio.ini`.

## C-Next (`.cnx`)
- `cnext_build.py` (PlatformIO pre-script) transpiles each listed `.cnx` → `.c`/`.h`
  in `include/`; add new modules to its list.
- Pure modules emit C-mode `.c`/`.h` with `extern "C"` guards (call from C++ as
  `Scope_method()`). Generated `.c`/`.h` are committed; regeneration is deterministic.

## Host telemetry tool (`tools/ovgt-telemetry/`)
- Node + npm + tsx + vitest + Ink + serialport + mongodb. **Not** bun, **not** OpenTUI
  (both need bun/Node≥26.3; serialport is only solid on plain Node).
- `npm test` · `npm run typecheck` · `npm start -- [--port X] [--label X] [--replay file] [--no-mongo]`
- `store`/`replay` tests need local MongoDB at `127.0.0.1:27017`.
- ink-testing-library: `useInput` subscribes in a post-render effect — tests must
  `await` a tick after `render()` and between keystrokes.
