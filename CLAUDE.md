# OVGT — Project Notes for Claude

Variable-geometry turbo controller. Teensy 4.1 / PlatformIO / Arduino, migrating
C++ → C-Next. (C-Next syntax lives in the global ~/.claude/CLAUDE.md.)

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
