# ovgt-telemetry

Live TUI + MongoDB logger for the OVGT Teensy telemetry stream (Part 2 of the
telemetry feature; the firmware emits 10 Hz NDJSON — see
`docs/plans/2026-06-19-mongodb-telemetry-design.md`).

## Requirements
- Node.js ≥ 20 (developed on 24), npm
- MongoDB at `mongodb://127.0.0.1:27017` (override with `--mongo`)
- Teensy on USB (auto-detected by VID `16c0`, or pass `--port`)

## Run
```bash
npm install
npm start                      # auto-detect port, log to mongodb ovgt db
npm start -- --no-mongo        # live view only (no logging)
npm start -- --port /dev/ttyACM0 --label drive-1
npm start -- --replay capture.ndjson   # offline: load a captured file into Mongo
```

## Keys
| key | action |
|---|---|
| `[` `]` | BPR target ∓ / ± 0.05 |
| `,` `.` | kp − / + 1 |
| `;` `'` | ki − / + 1 |
| digits + Enter | set vane % (0–100) |
| `a` | auto mode |
| `p` | print params |
| `l` | set/relabel session (type, Enter) |
| `q` | quit |

## MongoDB (`ovgt` db)
- `telemetry` — one doc per 10 Hz sample (+ host `ts`, `sessionId`)
- `settle_events` — one doc per `~`-gate measurement
- `sessions` — `{ _id, startTs, endTs, label, host }`
- both data collections indexed `{ sessionId: 1, t_ms: 1 }`

## Test
```bash
npm test          # vitest (store/replay tests need local MongoDB)
npm run typecheck
```
