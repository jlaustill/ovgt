# OVGT Telemetry Host (TUI + MongoDB) — Implementation Plan (Part 2 of 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Node.js terminal tool that reads the Teensy's 10 Hz NDJSON telemetry over USB serial, shows it live, logs every sample + settle event to MongoDB, and sends tuning commands back — replacing "paste serial text and grep it" and enabling offline replay of the `~` settled-CE gate.

**Architecture:** Pure, unit-tested core modules (line parser, formatter, keymap, port picker) wrapped by thin I/O shells (a `serialport` link, a `mongodb` store, an Ink UI). An `index.tsx` orchestrator wires serial → parse → { Ink view, Mongo insert } and maps keypresses → serial writes. A `--replay <file>` mode feeds a captured NDJSON file through the exact same path (the offline gate-replay capability).

**Tech Stack:** TypeScript on **Node.js 24 + npm** (no bun), run with **tsx**, tested with **vitest**. TUI = **Ink** (React for CLIs). Serial = **serialport** + **@serialport/parser-readline**. DB = **mongodb** driver. (Runtime decided 2026-06-21: bun + OpenTUI were rejected — OpenTUI's renderer needs bun/Node ≥26.3, while serialport is only rock-solid on plain Node.)

**Spec:** `docs/plans/2026-06-19-mongodb-telemetry-design.md`. **Part 1 (firmware)** is implemented and truck-verified (10 Hz `"t"` telemetry + `"s"` settle NDJSON; commits on `feature/mongodb-telemetry`).

**Prerequisites (already confirmed on this machine):**
- Node `v24.16.0`, npm `11.13.0`.
- MongoDB reachable at `mongodb://127.0.0.1:27017` (the store unit test uses a throwaway DB there).
- Teensy enumerates at `/dev/ttyACM0`, VID:PID `16c0:0483`.

**Wire format the firmware actually emits** (verified 2026-06-21 — use these as fixtures verbatim):
```
{"type":"t","t_ms":46649,"mode":"auto","cop_hpa":871,"cip_hpa":875,"boost_psi":0.00,"br":1.00,"tip_psi":0.00,"bpr":0.00,"bpr_target":1.00,"cit_c":20.32,"cot_c":20.91,"tit_c":20,"ce_pct":-1.00,"ce_settled":false,"dem_pct":25,"pos_pct":27,"brake":false}
{"type":"s","t_ms":99999,"tau_s":5.20,"settle_s":8.10,"step_c":42.30,"cot_slope_c_s":0.05,"boost_slope_psi_s":0.01,"settle_timer_s":2.30}
```
(The `"s"` line above is synthetic — no settle event was captured at cold idle — but its field set/order matches `ovgt.cpp` exactly.)

---

## File Structure

All under `tools/ovgt-telemetry/` (in the ovgt repo, per the spec):

| File | Responsibility |
|---|---|
| `package.json`, `tsconfig.json`, `vitest.config.ts` | Toolchain config |
| `src/types.ts` | `TelemetrySample`, `SettleEvent`, `ParsedLine` union |
| `src/parse.ts` (+ `.test.ts`) | **pure**: one raw line → `ParsedLine` (telemetry / settle / log) |
| `src/format.ts` (+ `.test.ts`) | **pure**: a `TelemetrySample` → display line strings |
| `src/keymap.ts` (+ `.test.ts`) | **pure**: a keypress + tuning view → serial command or local action |
| `src/serial.ts` (+ `.test.ts`) | `pickTeensyPort` (**pure**) + `SerialLink` (thin I/O, dependency-injected) |
| `src/store.ts` (+ `.test.ts`) | `MongoStore`: session + telemetry + settle inserts, indexes |
| `src/app.tsx` (+ `.test.tsx`) | Ink presentational `<App>` (panel + log tail + key handling) |
| `src/index.tsx` | orchestrator: arg parse, `<Root>` subscription wrapper, `main()` |
| `src/fixtures/telemetry.ndjson` | real captured lines for parser/replay tests |
| `README.md` | usage, keymap, collections, replay |

**Design rule:** all branching logic lives in the **pure** modules (`parse`, `format`, `keymap`, `pickTeensyPort`). The shells (`serial`, `store`, `app`, `index`) only do I/O and call the pure functions, so the logic is fully unit-tested without hardware, a terminal, or a live socket.

---

### Task 1: Scaffold the project

**Files:**
- Create: `tools/ovgt-telemetry/package.json`
- Create: `tools/ovgt-telemetry/tsconfig.json`
- Create: `tools/ovgt-telemetry/vitest.config.ts`
- Create: `tools/ovgt-telemetry/.gitignore`
- Create: `tools/ovgt-telemetry/src/smoke.test.ts`

- [ ] **Step 1: Create `package.json`**

```json
{
  "name": "ovgt-telemetry",
  "version": "0.1.0",
  "private": true,
  "type": "module",
  "engines": { "node": ">=20" },
  "scripts": {
    "start": "tsx src/index.tsx",
    "test": "vitest run",
    "test:watch": "vitest",
    "typecheck": "tsc --noEmit"
  },
  "dependencies": {
    "@serialport/parser-readline": "^12.0.0",
    "ink": "^5.0.1",
    "mongodb": "^6.8.0",
    "react": "^18.3.1",
    "serialport": "^12.0.0"
  },
  "devDependencies": {
    "@types/node": "^22.5.0",
    "@types/react": "^18.3.0",
    "ink-testing-library": "^4.0.0",
    "tsx": "^4.19.0",
    "typescript": "^5.5.0",
    "vitest": "^2.0.5"
  }
}
```

- [ ] **Step 2: Create `tsconfig.json`**

```json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "jsx": "react-jsx",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "resolveJsonModule": true,
    "types": ["node"],
    "noEmit": true
  },
  "include": ["src"]
}
```

- [ ] **Step 3: Create `vitest.config.ts`** (esbuild handles TSX/JSX; `jsx: 'automatic'` avoids React-import errors)

```typescript
import { defineConfig } from "vitest/config";

export default defineConfig({
  esbuild: { jsx: "automatic" },
  test: {
    environment: "node",
    include: ["src/**/*.test.{ts,tsx}"],
  },
});
```

- [ ] **Step 4: Create `.gitignore`**

```
node_modules/
*.log
```

- [ ] **Step 5: Create `src/smoke.test.ts`**

```typescript
import { expect, test } from "vitest";

test("toolchain runs", () => {
  expect(1 + 1).toBe(2);
});
```

- [ ] **Step 6: Install + run**

```bash
cd tools/ovgt-telemetry
npm install
npm test
```
Expected: vitest runs, `smoke.test.ts` PASSES (1 test).

- [ ] **Step 7: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/package.json tools/ovgt-telemetry/package-lock.json tools/ovgt-telemetry/tsconfig.json tools/ovgt-telemetry/vitest.config.ts tools/ovgt-telemetry/.gitignore tools/ovgt-telemetry/src/smoke.test.ts
git commit -m "feat(telemetry-host): scaffold Node+Ink+vitest project"
```

---

### Task 2: Types + line parser

**Files:**
- Create: `tools/ovgt-telemetry/src/types.ts`
- Create: `tools/ovgt-telemetry/src/fixtures/telemetry.ndjson`
- Create: `tools/ovgt-telemetry/src/parse.ts`
- Create: `tools/ovgt-telemetry/src/parse.test.ts`

- [ ] **Step 1: Create `src/types.ts`**

```typescript
export interface TelemetrySample {
  type: "t";
  t_ms: number;
  mode: "auto" | "manual" | "brake";
  cop_hpa: number;
  cip_hpa: number;
  boost_psi: number;
  br: number;
  tip_psi: number;
  bpr: number;
  bpr_target: number;
  cit_c: number;
  cot_c: number;
  tit_c: number;
  ce_pct: number;
  ce_settled: boolean;
  dem_pct: number;
  pos_pct: number;
  brake: boolean;
}

export interface SettleEvent {
  type: "s";
  t_ms: number;
  tau_s: number;
  settle_s: number;
  step_c: number;
  cot_slope_c_s: number;
  boost_slope_psi_s: number;
  settle_timer_s: number;
}

export type ParsedLine =
  | { kind: "telemetry"; sample: TelemetrySample }
  | { kind: "settle"; event: SettleEvent }
  | { kind: "log"; text: string };
```

- [ ] **Step 2: Create `src/fixtures/telemetry.ndjson`** (real capture from the truck, 2026-06-21; last two lines are non-telemetry to test the log path)

```
{"type":"t","t_ms":46649,"mode":"auto","cop_hpa":871,"cip_hpa":875,"boost_psi":0.00,"br":1.00,"tip_psi":0.00,"bpr":0.00,"bpr_target":1.00,"cit_c":20.32,"cot_c":20.91,"tit_c":20,"ce_pct":-1.00,"ce_settled":false,"dem_pct":25,"pos_pct":27,"brake":false}
{"type":"t","t_ms":50599,"mode":"auto","cop_hpa":871,"cip_hpa":875,"boost_psi":0.00,"br":1.00,"tip_psi":0.00,"bpr":0.00,"bpr_target":1.00,"cit_c":20.32,"cot_c":20.95,"tit_c":20,"ce_pct":-1.00,"ce_settled":false,"dem_pct":25,"pos_pct":27,"brake":false}
{"type":"s","t_ms":99999,"tau_s":5.20,"settle_s":8.10,"step_c":42.30,"cot_slope_c_s":0.05,"boost_slope_psi_s":0.01,"settle_timer_s":2.30}
Setup complete
BPR target = 1.00
```

- [ ] **Step 3: Write the failing test** — `src/parse.test.ts`

```typescript
import { expect, test } from "vitest";
import { parseLine } from "./parse";

const telemetry =
  '{"type":"t","t_ms":46649,"mode":"auto","cop_hpa":871,"cip_hpa":875,"boost_psi":0.00,"br":1.00,"tip_psi":0.00,"bpr":0.00,"bpr_target":1.00,"cit_c":20.32,"cot_c":20.91,"tit_c":20,"ce_pct":-1.00,"ce_settled":false,"dem_pct":25,"pos_pct":27,"brake":false}';
const settle =
  '{"type":"s","t_ms":99999,"tau_s":5.20,"settle_s":8.10,"step_c":42.30,"cot_slope_c_s":0.05,"boost_slope_psi_s":0.01,"settle_timer_s":2.30}';

test("parses a telemetry line", () => {
  const r = parseLine(telemetry);
  expect(r.kind).toBe("telemetry");
  if (r.kind === "telemetry") {
    expect(r.sample.t_ms).toBe(46649);
    expect(r.sample.mode).toBe("auto");
    expect(r.sample.cot_c).toBeCloseTo(20.91);
    expect(r.sample.ce_settled).toBe(false);
  }
});

test("parses a settle line", () => {
  const r = parseLine(settle);
  expect(r.kind).toBe("settle");
  if (r.kind === "settle") {
    expect(r.event.tau_s).toBeCloseTo(5.2);
    expect(r.event.settle_timer_s).toBeCloseTo(2.3);
  }
});

test("non-JSON banner becomes a log line", () => {
  const r = parseLine("Setup complete");
  expect(r).toEqual({ kind: "log", text: "Setup complete" });
});

test("JSON without a known type becomes a log line", () => {
  const r = parseLine('{"foo":1}');
  expect(r.kind).toBe("log");
});

test("blank line becomes an empty log line", () => {
  expect(parseLine("  \r")).toEqual({ kind: "log", text: "" });
});
```

- [ ] **Step 4: Run, expect FAIL**

```bash
cd tools/ovgt-telemetry && npx vitest run src/parse.test.ts
```
Expected: FAIL — `parse.ts` / `parseLine` does not exist.

- [ ] **Step 5: Create `src/parse.ts`**

```typescript
import type { ParsedLine, SettleEvent, TelemetrySample } from "./types";

export function parseLine(line: string): ParsedLine {
  const trimmed = line.trim();
  if (trimmed === "") return { kind: "log", text: "" };

  let obj: unknown;
  try {
    obj = JSON.parse(trimmed);
  } catch {
    return { kind: "log", text: trimmed };
  }

  if (obj === null || typeof obj !== "object") {
    return { kind: "log", text: trimmed };
  }

  const type = (obj as { type?: unknown }).type;
  if (type === "t") return { kind: "telemetry", sample: obj as TelemetrySample };
  if (type === "s") return { kind: "settle", event: obj as SettleEvent };
  return { kind: "log", text: trimmed };
}
```

- [ ] **Step 6: Run, expect PASS** (`npx vitest run src/parse.test.ts` → 5 passed)

- [ ] **Step 7: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/types.ts tools/ovgt-telemetry/src/parse.ts tools/ovgt-telemetry/src/parse.test.ts tools/ovgt-telemetry/src/fixtures/telemetry.ndjson
git commit -m "feat(telemetry-host): NDJSON line parser + types + real fixtures"
```

---

### Task 3: Telemetry formatter

**Files:**
- Create: `tools/ovgt-telemetry/src/format.ts`
- Create: `tools/ovgt-telemetry/src/format.test.ts`

- [ ] **Step 1: Write the failing test** — `src/format.test.ts`

```typescript
import { expect, test } from "vitest";
import { formatTelemetry } from "./format";
import type { TelemetrySample } from "./types";

const sample: TelemetrySample = {
  type: "t", t_ms: 46649, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  cit_c: 20.32, cot_c: 20.91, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};

test("formats core telemetry fields", () => {
  const lines = formatTelemetry(sample).join("\n");
  expect(lines).toContain("mode auto");
  expect(lines).toContain("COT 20.9C");
  expect(lines).toContain("BPR 0.00/1.00");
  expect(lines).toContain("dem 25%");
});

test("ce_pct -1 renders as -- and settled drops the ~", () => {
  expect(formatTelemetry(sample).join("\n")).toContain("CE --");
  const warm = { ...sample, ce_pct: 71, ce_settled: false };
  expect(formatTelemetry(warm).join("\n")).toContain("CE 71%~");
  const settled = { ...warm, ce_settled: true };
  expect(formatTelemetry(settled).join("\n")).toContain("CE 71%");
  expect(formatTelemetry(settled).join("\n")).not.toContain("71%~");
});
```

- [ ] **Step 2: Run, expect FAIL** (`npx vitest run src/format.test.ts`)

- [ ] **Step 3: Create `src/format.ts`**

```typescript
import type { TelemetrySample } from "./types";

export function formatTelemetry(s: TelemetrySample): string[] {
  const ce = s.ce_pct < 0 ? "--" : `${s.ce_pct.toFixed(0)}%${s.ce_settled ? "" : "~"}`;
  return [
    `mode ${s.mode}   brake ${s.brake ? "ON" : "off"}`,
    `boost ${s.boost_psi.toFixed(1)}psi  BR ${s.br.toFixed(2)}  TIP ${s.tip_psi.toFixed(1)}psi`,
    `BPR ${s.bpr.toFixed(2)}/${s.bpr_target.toFixed(2)}   dem ${s.dem_pct}%  pos ${s.pos_pct}%`,
    `CIT ${s.cit_c.toFixed(1)}C  COT ${s.cot_c.toFixed(1)}C  TIT ${s.tit_c}C`,
    `CE ${ce}`,
  ];
}
```

- [ ] **Step 4: Run, expect PASS** (2 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/format.ts tools/ovgt-telemetry/src/format.test.ts
git commit -m "feat(telemetry-host): telemetry display formatter"
```

---

### Task 4: Keymap (pure)

**Files:**
- Create: `tools/ovgt-telemetry/src/keymap.ts`
- Create: `tools/ovgt-telemetry/src/keymap.test.ts`

The firmware accepts absolute commands: `auto`, `params`, `bpr <v>`, `kp <v>`, `ki <v>`, `<0-100>`. `bpr_target` comes back in every telemetry sample (always seed from live), so `[`/`]` adjust the *live* target; `kp`/`ki` are not in the stream, so the host keeps them as local state. Keys: `[`/`]` bpr ∓0.05, `,`/`.` kp ∓1, `;`/`'` ki ∓1, `a` auto, `p` params, `l` relabel, `q` quit (all unshifted — no modifier handling needed).

- [ ] **Step 1: Write the failing test** — `src/keymap.test.ts`

```typescript
import { expect, test } from "vitest";
import { handleKey, type TuningView } from "./keymap";

const view: TuningView = { bprTarget: 1.0, kp: 20, ki: 20 };

test("] raises bpr target by 0.05 (absolute command + new state)", () => {
  const r = handleKey("]", view);
  expect(r).toEqual({ type: "command", command: "bpr 1.05", tuning: { bprTarget: 1.05, kp: 20, ki: 20 } });
});

test("[ lowers bpr target by 0.05 with clean rounding", () => {
  const r = handleKey("[", view);
  expect(r).toEqual({ type: "command", command: "bpr 0.95", tuning: { bprTarget: 0.95, kp: 20, ki: 20 } });
});

test(". raises kp, , lowers kp", () => {
  expect(handleKey(".", view)).toMatchObject({ command: "kp 21" });
  expect(handleKey(",", view)).toMatchObject({ command: "kp 19" });
});

test("' raises ki, ; lowers ki", () => {
  expect(handleKey("'", view)).toMatchObject({ command: "ki 21" });
  expect(handleKey(";", view)).toMatchObject({ command: "ki 19" });
});

test("a and p are commands; l and q are local actions", () => {
  expect(handleKey("a", view)).toEqual({ type: "command", command: "auto", tuning: view });
  expect(handleKey("p", view)).toEqual({ type: "command", command: "params", tuning: view });
  expect(handleKey("l", view)).toEqual({ type: "action", action: "relabel" });
  expect(handleKey("q", view)).toEqual({ type: "action", action: "quit" });
});

test("unmapped key does nothing", () => {
  expect(handleKey("z", view)).toEqual({ type: "none" });
});
```

- [ ] **Step 2: Run, expect FAIL** (`npx vitest run src/keymap.test.ts`)

- [ ] **Step 3: Create `src/keymap.ts`**

```typescript
export interface TuningView {
  bprTarget: number;
  kp: number;
  ki: number;
}

export type KeyResult =
  | { type: "command"; command: string; tuning: TuningView }
  | { type: "action"; action: "relabel" | "quit" }
  | { type: "none" };

const round2 = (n: number): number => Math.round(n * 100) / 100;

export function handleKey(input: string, t: TuningView): KeyResult {
  switch (input) {
    case "[": {
      const v = round2(t.bprTarget - 0.05);
      return { type: "command", command: `bpr ${v}`, tuning: { ...t, bprTarget: v } };
    }
    case "]": {
      const v = round2(t.bprTarget + 0.05);
      return { type: "command", command: `bpr ${v}`, tuning: { ...t, bprTarget: v } };
    }
    case ",": {
      const v = t.kp - 1;
      return { type: "command", command: `kp ${v}`, tuning: { ...t, kp: v } };
    }
    case ".": {
      const v = t.kp + 1;
      return { type: "command", command: `kp ${v}`, tuning: { ...t, kp: v } };
    }
    case ";": {
      const v = t.ki - 1;
      return { type: "command", command: `ki ${v}`, tuning: { ...t, ki: v } };
    }
    case "'": {
      const v = t.ki + 1;
      return { type: "command", command: `ki ${v}`, tuning: { ...t, ki: v } };
    }
    case "a":
      return { type: "command", command: "auto", tuning: t };
    case "p":
      return { type: "command", command: "params", tuning: t };
    case "l":
      return { type: "action", action: "relabel" };
    case "q":
      return { type: "action", action: "quit" };
    default:
      return { type: "none" };
  }
}
```

- [ ] **Step 4: Run, expect PASS** (6 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/keymap.ts tools/ovgt-telemetry/src/keymap.test.ts
git commit -m "feat(telemetry-host): pure keymap (tuning keys -> serial commands)"
```

---

### Task 5: Serial link

**Files:**
- Create: `tools/ovgt-telemetry/src/serial.ts`
- Create: `tools/ovgt-telemetry/src/serial.test.ts`

`pickTeensyPort` is pure (testable). `SerialLink` is dependency-injected: it takes an `open()` factory returning a minimal `SerialStream`, so the test injects a fake stream (a real `PassThrough` for incoming bytes + a write spy) and the orchestrator injects a real `SerialPort`.

- [ ] **Step 1: Write the failing test** — `src/serial.test.ts`

```typescript
import { EventEmitter } from "node:events";
import { PassThrough } from "node:stream";
import { expect, test, vi } from "vitest";
import { pickTeensyPort, SerialLink, type SerialStream } from "./serial";

test("pickTeensyPort finds the 16c0 device, case-insensitive", () => {
  const ports = [
    { path: "/dev/ttyS0", vendorId: undefined },
    { path: "/dev/ttyACM0", vendorId: "16C0", productId: "0483" },
  ];
  expect(pickTeensyPort(ports)).toBe("/dev/ttyACM0");
  expect(pickTeensyPort([{ path: "/dev/ttyS0" }])).toBeUndefined();
});

// Fake serial stream: inner PassThrough feeds `pipe`; writes are recorded.
class FakeSerial extends EventEmitter implements SerialStream {
  readable = new PassThrough();
  written: string[] = [];
  pipe<T extends NodeJS.WritableStream>(dest: T): T {
    return this.readable.pipe(dest);
  }
  write(data: string): boolean {
    this.written.push(data);
    return true;
  }
  inject(data: string): void {
    this.readable.write(data);
  }
}

test("emits parsed lines split on newline", async () => {
  const fake = new FakeSerial();
  const lines: string[] = [];
  const link = new SerialLink({ open: () => fake, onLine: (l) => lines.push(l) });
  link.start();
  fake.inject('{"type":"t"}\nSetup ');
  fake.inject("complete\n");
  await new Promise((r) => setTimeout(r, 10));
  expect(lines).toEqual(['{"type":"t"}', "Setup complete"]);
});

test("send() appends a newline once", () => {
  const fake = new FakeSerial();
  const link = new SerialLink({ open: () => fake, onLine: () => {} });
  link.start();
  link.send("bpr 1.05");
  link.send("auto\n");
  expect(fake.written).toEqual(["bpr 1.05\n", "auto\n"]);
});

test("reconnects after an unexpected close", async () => {
  const open = vi.fn(() => new FakeSerial());
  const link = new SerialLink({ open, onLine: () => {}, reconnectMs: 5 });
  link.start();
  (open.mock.results[0]!.value as FakeSerial).emit("close");
  await new Promise((r) => setTimeout(r, 20));
  expect(open).toHaveBeenCalledTimes(2);
});
```

- [ ] **Step 2: Run, expect FAIL** (`npx vitest run src/serial.test.ts`)

- [ ] **Step 3: Create `src/serial.ts`**

```typescript
import { ReadlineParser } from "@serialport/parser-readline";

export interface PortLike {
  path: string;
  vendorId?: string;
  productId?: string;
}

const TEENSY_VENDOR_ID = "16c0";

export function pickTeensyPort(ports: PortLike[]): string | undefined {
  const teensy = ports.find((p) => (p.vendorId ?? "").toLowerCase() === TEENSY_VENDOR_ID);
  return teensy?.path;
}

// Minimal surface shared by a real SerialPort and the test's fake stream.
export interface SerialStream {
  pipe<T extends NodeJS.WritableStream>(dest: T): T;
  write(data: string): boolean;
  on(event: "close", cb: () => void): this;
  on(event: "error", cb: (err: Error) => void): this;
}

export interface SerialLinkOptions {
  open: () => SerialStream;
  onLine: (line: string) => void;
  onStatus?: (status: "open" | "closed" | "error", detail?: string) => void;
  reconnectMs?: number;
}

export class SerialLink {
  private port?: SerialStream;
  private stopped = false;

  constructor(private readonly opts: SerialLinkOptions) {}

  start(): void {
    this.stopped = false;
    this.connect();
  }

  private connect(): void {
    const port = this.opts.open();
    this.port = port;
    const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
    parser.on("data", (line: string) => this.opts.onLine(line));
    port.on("error", (err) => this.opts.onStatus?.("error", err.message));
    port.on("close", () => {
      this.opts.onStatus?.("closed");
      if (!this.stopped) {
        setTimeout(() => this.connect(), this.opts.reconnectMs ?? 2000);
      }
    });
    this.opts.onStatus?.("open");
  }

  send(command: string): void {
    this.port?.write(command.endsWith("\n") ? command : `${command}\n`);
  }

  stop(): void {
    this.stopped = true;
  }
}
```

- [ ] **Step 4: Run, expect PASS** (4 passed)

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/serial.ts tools/ovgt-telemetry/src/serial.test.ts
git commit -m "feat(telemetry-host): serial link (port picker + readline + reconnect)"
```

---

### Task 6: Ink app (presentational)

**Files:**
- Create: `tools/ovgt-telemetry/src/app.tsx`
- Create: `tools/ovgt-telemetry/src/app.test.tsx`

`<App>` is presentational: it receives the latest sample/settle/logs/status via props and reports user intent via `onCommand`/`onRelabel`. It owns only transient UI state (local kp/ki, the vane digit buffer, the label-entry buffer). All command decisions go through the pure `handleKey`.

- [ ] **Step 1: Write the failing test** — `src/app.test.tsx`

```tsx
import { render } from "ink-testing-library";
import React from "react";
import { expect, test } from "vitest";
import { App } from "./app";
import type { TelemetrySample } from "./types";

const sample: TelemetrySample = {
  type: "t", t_ms: 1, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  cit_c: 20.3, cot_c: 20.9, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};
const tick = () => new Promise((r) => setTimeout(r, 20));

test("renders the telemetry panel", () => {
  const { lastFrame } = render(
    <App sample={sample} logs={["Setup complete"]} sessionLabel="drive-1" status="open" onCommand={() => {}} onRelabel={() => {}} />,
  );
  expect(lastFrame()).toContain("COT 20.9C");
  expect(lastFrame()).toContain("drive-1");
  expect(lastFrame()).toContain("Setup complete");
});

test("pressing ] sends an absolute bpr command", async () => {
  const cmds: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={(c) => cmds.push(c)} onRelabel={() => {}} />,
  );
  stdin.write("]");
  await tick();
  expect(cmds).toContain("bpr 1.05");
});

test("digits then Enter send a clamped vane %", async () => {
  const cmds: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={(c) => cmds.push(c)} onRelabel={() => {}} />,
  );
  stdin.write("4");
  stdin.write("5");
  stdin.write("\r");
  await tick();
  expect(cmds).toContain("45");
});

test("l then typing then Enter relabels", async () => {
  const labels: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={() => {}} onRelabel={(l) => labels.push(l)} />,
  );
  stdin.write("l");
  stdin.write("h");
  stdin.write("i");
  stdin.write("\r");
  await tick();
  expect(labels).toEqual(["hi"]);
});
```

- [ ] **Step 2: Run, expect FAIL** (`npx vitest run src/app.test.tsx`)

- [ ] **Step 3: Create `src/app.tsx`**

```tsx
import { Box, Text, useApp, useInput } from "ink";
import React from "react";
import { formatTelemetry } from "./format";
import { handleKey, type TuningView } from "./keymap";
import type { SettleEvent, TelemetrySample } from "./types";

export interface AppProps {
  sample?: TelemetrySample;
  settle?: SettleEvent;
  logs: string[];
  sessionLabel: string;
  status: string;
  onCommand: (command: string) => void;
  onRelabel: (label: string) => void;
}

export function App(props: AppProps): React.ReactElement {
  const { sample, settle, logs, sessionLabel, status, onCommand, onRelabel } = props;
  const { exit } = useApp();
  const [tuning, setTuning] = React.useState({ kp: 20, ki: 20 });
  const [vaneBuf, setVaneBuf] = React.useState("");
  const [labelMode, setLabelMode] = React.useState(false);
  const [labelBuf, setLabelBuf] = React.useState("");

  useInput((input, key) => {
    if (labelMode) {
      if (key.return) {
        onRelabel(labelBuf);
        setLabelMode(false);
        setLabelBuf("");
      } else if (key.escape) {
        setLabelMode(false);
        setLabelBuf("");
      } else if (key.backspace || key.delete) {
        setLabelBuf((b) => b.slice(0, -1));
      } else if (input) {
        setLabelBuf((b) => b + input);
      }
      return;
    }

    if (/^[0-9]$/.test(input)) {
      setVaneBuf((b) => (b + input).slice(0, 3));
      return;
    }
    if (key.return) {
      if (vaneBuf) {
        onCommand(String(Math.min(100, parseInt(vaneBuf, 10))));
        setVaneBuf("");
      }
      return;
    }
    if (key.escape) {
      setVaneBuf("");
      return;
    }

    const view: TuningView = { bprTarget: sample?.bpr_target ?? 1.0, kp: tuning.kp, ki: tuning.ki };
    const res = handleKey(input, view);
    if (res.type === "command") {
      onCommand(res.command);
      setTuning({ kp: res.tuning.kp, ki: res.tuning.ki });
    } else if (res.type === "action") {
      if (res.action === "relabel") setLabelMode(true);
      else if (res.action === "quit") exit();
    }
  });

  const lines = sample ? formatTelemetry(sample) : ["(waiting for telemetry…)"];

  return (
    <Box flexDirection="column">
      <Box borderStyle="round" flexDirection="column" paddingX={1}>
        <Text bold>
          OVGT [{sessionLabel}] {status} kp {tuning.kp} ki {tuning.ki}
        </Text>
        {lines.map((l, i) => (
          <Text key={i}>{l}</Text>
        ))}
        {settle && (
          <Text color="cyan">
            last settle: τ {settle.tau_s.toFixed(1)}s step {settle.step_c.toFixed(1)}C slope {settle.cot_slope_c_s.toFixed(2)}C/s
          </Text>
        )}
        {vaneBuf && <Text color="yellow">vane → {vaneBuf} (Enter to send)</Text>}
        {labelMode && <Text color="green">label → {labelBuf}_ (Enter to save, Esc cancel)</Text>}
      </Box>
      <Box flexDirection="column" paddingX={1}>
        {logs.slice(-6).map((l, i) => (
          <Text key={i} dimColor>
            {l}
          </Text>
        ))}
      </Box>
      <Text dimColor>[ ] bpr  , . kp  ; ' ki  a auto  p params  digits+Enter vane  l label  q quit</Text>
    </Box>
  );
}
```

- [ ] **Step 4: Run, expect PASS** (4 passed). If JSX/React errors appear under vitest, confirm `esbuild: { jsx: "automatic" }` is in `vitest.config.ts`.

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/app.tsx tools/ovgt-telemetry/src/app.test.tsx
git commit -m "feat(telemetry-host): Ink live panel + log tail + key handling"
```

---

### Task 7: MongoDB store

**Files:**
- Create: `tools/ovgt-telemetry/src/store.ts`
- Create: `tools/ovgt-telemetry/src/store.test.ts`

The store unit test runs against the **local MongoDB** at `127.0.0.1:27017` (confirmed running) using a unique throwaway database that it drops afterward — it never touches the real `ovgt` DB.

- [ ] **Step 1: Write the failing test** — `src/store.test.ts`

```typescript
import { MongoClient } from "mongodb";
import { afterAll, beforeAll, expect, test } from "vitest";
import { MongoStore } from "./store";
import type { SettleEvent, TelemetrySample } from "./types";

const uri = "mongodb://127.0.0.1:27017";
const dbName = `ovgt_test_${Date.now()}`;

const sample: TelemetrySample = {
  type: "t", t_ms: 100, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  cit_c: 20.3, cot_c: 20.9, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};
const settle: SettleEvent = {
  type: "s", t_ms: 200, tau_s: 5.2, settle_s: 8.1, step_c: 42.3,
  cot_slope_c_s: 0.05, boost_slope_psi_s: 0.01, settle_timer_s: 2.3,
};

afterAll(async () => {
  const client = new MongoClient(uri);
  await client.connect();
  await client.db(dbName).dropDatabase();
  await client.close();
});

test("connect creates a session and inserts telemetry + settle with sessionId", async () => {
  const store = new MongoStore(uri, dbName);
  const sessionId = await store.connect("test-drive", "testhost");
  await store.insertTelemetry(sample);
  await store.insertSettle(settle);
  await store.relabel("relabeled");
  await store.close();

  const client = new MongoClient(uri);
  await client.connect();
  const db = client.db(dbName);
  const tdoc = await db.collection("telemetry").findOne({ sessionId });
  const sdoc = await db.collection("settle_events").findOne({ sessionId });
  const sess = await db.collection("sessions").findOne({ _id: sessionId });
  await client.close();

  expect(tdoc?.cot_c).toBeCloseTo(20.9);
  expect(tdoc?.ts).toBeInstanceOf(Date);
  expect(sdoc?.tau_s).toBeCloseTo(5.2);
  expect(sess?.label).toBe("relabeled");
  expect(sess?.endTs).toBeInstanceOf(Date);
});
```

- [ ] **Step 2: Run, expect FAIL** (`npx vitest run src/store.test.ts`)

- [ ] **Step 3: Create `src/store.ts`**

```typescript
import { type Db, MongoClient } from "mongodb";
import type { SettleEvent, TelemetrySample } from "./types";

export interface SessionDoc {
  _id: string;
  startTs: Date;
  endTs?: Date;
  label: string;
  host: string;
}

export class MongoStore {
  private readonly client: MongoClient;
  private db?: Db;
  private sessionId?: string;

  constructor(uri: string, private readonly dbName = "ovgt") {
    this.client = new MongoClient(uri);
  }

  async connect(label: string, host: string): Promise<string> {
    await this.client.connect();
    const db = this.client.db(this.dbName);
    this.db = db;
    await db.collection("telemetry").createIndex({ sessionId: 1, t_ms: 1 });
    await db.collection("settle_events").createIndex({ sessionId: 1, t_ms: 1 });

    const sessionId = `${host}-${Date.now()}`;
    this.sessionId = sessionId;
    const doc: SessionDoc = { _id: sessionId, startTs: new Date(), label, host };
    await db.collection<SessionDoc>("sessions").insertOne(doc);
    return sessionId;
  }

  async insertTelemetry(sample: TelemetrySample): Promise<void> {
    if (!this.db) return;
    await this.db.collection("telemetry").insertOne({ ...sample, ts: new Date(), sessionId: this.sessionId });
  }

  async insertSettle(event: SettleEvent): Promise<void> {
    if (!this.db) return;
    await this.db.collection("settle_events").insertOne({ ...event, ts: new Date(), sessionId: this.sessionId });
  }

  async relabel(label: string): Promise<void> {
    if (!this.db || !this.sessionId) return;
    await this.db.collection<SessionDoc>("sessions").updateOne({ _id: this.sessionId }, { $set: { label } });
  }

  async close(): Promise<void> {
    if (this.db && this.sessionId) {
      await this.db.collection<SessionDoc>("sessions").updateOne({ _id: this.sessionId }, { $set: { endTs: new Date() } });
    }
    await this.client.close();
  }
}
```

- [ ] **Step 4: Run, expect PASS** (1 passed). If it errors with a connection refusal, start MongoDB first (`mongosh --eval 'db.runCommand({ping:1})'` should return `ok: 1`).

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/store.ts tools/ovgt-telemetry/src/store.test.ts
git commit -m "feat(telemetry-host): MongoStore (sessions + telemetry + settle inserts)"
```

---

### Task 8: Orchestrator + live truck verify

**Files:**
- Create: `tools/ovgt-telemetry/src/index.tsx`

`index.tsx` parses args, builds the serial source (real port, autodetected or `--port`) or a `--replay` file source, the optional `MongoStore`, and a `<Root>` that subscribes to a small `EventEmitter` feed and renders `<App>`. Each line: `parseLine` → emit to the feed + insert to Mongo.

- [ ] **Step 1: Create `src/index.tsx`**

```tsx
import { EventEmitter } from "node:events";
import { readFileSync } from "node:fs";
import { hostname } from "node:os";
import { render } from "ink";
import React from "react";
import { SerialPort } from "serialport";
import { App } from "./app";
import { parseLine } from "./parse";
import { pickTeensyPort, SerialLink, type SerialStream } from "./serial";
import { MongoStore } from "./store";
import type { SettleEvent, TelemetrySample } from "./types";

interface Args {
  port?: string;
  mongo: string;
  noMongo: boolean;
  label: string;
  replay?: string;
}

function parseArgs(argv: string[]): Args {
  const args: Args = { mongo: "mongodb://127.0.0.1:27017", noMongo: false, label: "session" };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--port") args.port = argv[++i];
    else if (a === "--mongo") args.mongo = argv[++i]!;
    else if (a === "--no-mongo") args.noMongo = true;
    else if (a === "--label") args.label = argv[++i]!;
    else if (a === "--replay") args.replay = argv[++i];
  }
  return args;
}

type Feed = EventEmitter;

function Root({ feed, sendCommand, onRelabel, initialLabel }: {
  feed: Feed;
  sendCommand: (c: string) => void;
  onRelabel: (l: string) => void;
  initialLabel: string;
}): React.ReactElement {
  const [sample, setSample] = React.useState<TelemetrySample>();
  const [settle, setSettle] = React.useState<SettleEvent>();
  const [logs, setLogs] = React.useState<string[]>([]);
  const [label, setLabel] = React.useState(initialLabel);
  const [status, setStatus] = React.useState("connecting");

  React.useEffect(() => {
    feed.on("sample", (s: TelemetrySample) => setSample(s));
    feed.on("settle", (s: SettleEvent) => setSettle(s));
    feed.on("log", (l: string) => setLogs((prev) => [...prev, l].slice(-100)));
    feed.on("status", (s: string) => setStatus(s));
    return () => feed.removeAllListeners();
  }, [feed]);

  return (
    <App
      sample={sample}
      settle={settle}
      logs={logs}
      sessionLabel={label}
      status={status}
      onCommand={sendCommand}
      onRelabel={(l) => {
        setLabel(l);
        onRelabel(l);
      }}
    />
  );
}

async function main(): Promise<void> {
  const args = parseArgs(process.argv.slice(2));
  const store = args.noMongo ? undefined : new MongoStore(args.mongo);

  // Headless replay: load a captured NDJSON file into Mongo, awaiting every
  // insert, then print a summary and exit. This is the offline ~-gate replay
  // path; it is deterministic and does not start the TUI.
  if (args.replay) {
    if (store) await store.connect(args.label, hostname());
    let t = 0;
    let s = 0;
    let logs = 0;
    for (const line of readFileSync(args.replay, "utf8").split("\n")) {
      if (line.trim() === "") continue;
      const parsed = parseLine(line);
      if (parsed.kind === "telemetry") {
        await store?.insertTelemetry(parsed.sample);
        t++;
      } else if (parsed.kind === "settle") {
        await store?.insertSettle(parsed.event);
        s++;
      } else {
        logs++;
      }
    }
    await store?.close();
    process.stdout.write(`replayed ${args.replay}: ${t} telemetry, ${s} settle, ${logs} log lines\n`);
    return;
  }

  // Live: serial -> Ink TUI (+ optional Mongo).
  const feed: Feed = new EventEmitter();
  if (store) await store.connect(args.label, hostname());

  const handleLine = (line: string): void => {
    const parsed = parseLine(line);
    if (parsed.kind === "telemetry") {
      feed.emit("sample", parsed.sample);
      void store?.insertTelemetry(parsed.sample);
    } else if (parsed.kind === "settle") {
      feed.emit("settle", parsed.event);
      void store?.insertSettle(parsed.event);
    } else if (parsed.text !== "") {
      feed.emit("log", parsed.text);
    }
  };

  let portPath = args.port;
  if (!portPath) {
    portPath = pickTeensyPort(await SerialPort.list());
  }
  const link = new SerialLink({
    open: () => new SerialPort({ path: portPath ?? "/dev/ttyACM0", baudRate: 115200 }) as unknown as SerialStream,
    onLine: handleLine,
    onStatus: (st, d) => feed.emit("status", d ? `${st}: ${d}` : st),
  });

  render(
    <Root
      feed={feed}
      sendCommand={(c) => link.send(c)}
      onRelabel={(l) => void store?.relabel(l)}
      initialLabel={args.label}
    />,
  );

  // Start the source AFTER mount so Root's feed subscription is already live.
  setImmediate(() => {
    if (!portPath) feed.emit("log", "no Teensy found (VID 16c0); trying /dev/ttyACM0 — or pass --port");
    if (store) feed.emit("log", `mongo: logging to ${args.mongo} db=ovgt`);
    link.start();
  });

  const shutdown = async (): Promise<void> => {
    link.stop();
    await store?.close();
    process.exit(0);
  };
  process.on("SIGINT", () => void shutdown());
}

void main();
```

- [ ] **Step 2: Typecheck**

```bash
cd tools/ovgt-telemetry && npm run typecheck
```
Expected: no errors.

- [ ] **Step 3: Run the full unit suite** (`npm test`) — all prior tests still PASS.

- [ ] **Step 4: Truck verify — live view only first** (Teensy plugged in, engine running)

```bash
cd tools/ovgt-telemetry && npm start -- --no-mongo
```
Expected: the Ink panel updates ~10×/s with live values; boot/ack lines appear in the log tail; `]`/`[` change the BPR target on the truck (watch `BPR x/y`); `q` quits. This is the spec's "verify the live view against the truck" gate.

- [ ] **Step 5: Truck verify — with Mongo**

```bash
cd tools/ovgt-telemetry && npm start -- --label drive-1
# in another shell:
mongosh ovgt --quiet --eval 'db.telemetry.countDocuments()'   # grows ~10/s
```
Expected: telemetry count climbs; `db.sessions.findOne()` shows your label + `startTs`.

- [ ] **Step 6: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/index.tsx
git commit -m "feat(telemetry-host): orchestrator (serial/replay -> Ink + Mongo)"
```

---

### Task 9: Offline replay integration test

**Files:**
- Create: `tools/ovgt-telemetry/src/replay.test.ts`

Proves the parse → store path end-to-end against the captured fixture — and is exactly the offline `~`-gate replay capability the whole project is for.

- [ ] **Step 1: Write the failing test** — `src/replay.test.ts`

```typescript
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { MongoClient } from "mongodb";
import { afterAll, expect, test } from "vitest";
import { parseLine } from "./parse";
import { MongoStore } from "./store";

const uri = "mongodb://127.0.0.1:27017";
const dbName = `ovgt_replay_${Date.now()}`;
const fixture = fileURLToPath(new URL("./fixtures/telemetry.ndjson", import.meta.url));

afterAll(async () => {
  const client = new MongoClient(uri);
  await client.connect();
  await client.db(dbName).dropDatabase();
  await client.close();
});

test("replaying a captured NDJSON file lands telemetry + settle docs", async () => {
  const store = new MongoStore(uri, dbName);
  const sessionId = await store.connect("replay", "testhost");
  let logs = 0;
  for (const line of readFileSync(fixture, "utf8").split("\n")) {
    if (line.trim() === "") continue;
    const parsed = parseLine(line);
    if (parsed.kind === "telemetry") await store.insertTelemetry(parsed.sample);
    else if (parsed.kind === "settle") await store.insertSettle(parsed.event);
    else logs++;
  }
  await store.close();

  const client = new MongoClient(uri);
  await client.connect();
  const db = client.db(dbName);
  const tCount = await db.collection("telemetry").countDocuments({ sessionId });
  const sCount = await db.collection("settle_events").countDocuments({ sessionId });
  await client.close();

  expect(tCount).toBe(2); // two "t" lines in the fixture
  expect(sCount).toBe(1); // one "s" line
  expect(logs).toBe(2);   // "Setup complete" + "BPR target = 1.00"
});
```

- [ ] **Step 2: Run, expect FAIL** then PASS once the fixture path resolves (it was created in Task 2). Run:

```bash
cd tools/ovgt-telemetry && npx vitest run src/replay.test.ts
```
Expected: PASS (1 test).

- [ ] **Step 3: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/src/replay.test.ts
git commit -m "test(telemetry-host): offline NDJSON replay -> Mongo integration"
```

---

### Task 10: README

**Files:**
- Create: `tools/ovgt-telemetry/README.md`

- [ ] **Step 1: Create `README.md`**

````markdown
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
npm start -- --replay capture.ndjson   # offline: replay a captured file
```

## Keys
| key | action |
|---|---|
| `[` `]` | BPR target ∓ / ± 0.05 |
| `,` `.` | kp − / + 1 |
| `;` `'` | ki − / + 1 |
| digits + Enter | set vane % (0–100) |
| `a` | auto mode | `p` | print params |
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
````

- [ ] **Step 2: Commit**

```bash
cd /home/linux/code/ovgt
git add tools/ovgt-telemetry/README.md
git commit -m "docs(telemetry-host): README (usage, keymap, collections)"
```

---

## Self-Review

**Spec coverage (against `2026-06-19-mongodb-telemetry-design.md`):**
- Component 3 host: serial auto-detect (`pickTeensyPort`, Task 5) ✓; `--port` (Task 8) ✓; 115200 readline (Task 5) ✓; `JSON.parse` route by type + log-pane fallback (Task 2, 8) ✓; Ink live view + log pane (Task 6) ✓; tuning keymap + relabel keypress (Task 4, 6) ✓; Mongo insert + sessions + relabel + endTs (Task 7, 8) ✓.
- Component 4 schema: `telemetry` / `settle_events` / `sessions`, host-owned stamping, `{sessionId,t_ms}` indexes (Task 7) ✓.
- Data flow + error handling: reconnect loop (Task 5) ✓; malformed line → log pane (Task 2) ✓; Mongo failures don't block the view (`void store?.insert…`, Task 8) ✓.
- Testing: parser unit (Task 2), fake-serial fixture (Task 5), Mongo insert against local deployment (Task 7), offline replay (Task 9) ✓.
- Build order: units first (Tasks 2–7), then wire + **truck-verify the live view before/with Mongo** (Task 8) — matches the spec's step 4→5→6 staging via the `--no-mongo` then full run.
- Keymap deviation from spec draft: spec floated `<`/`>` for kp; this plan uses unshifted `,`/`.` (kp) and `;`/`'` (ki) to avoid modifier handling. `kp`/`ki` are tracked locally (not in the telemetry stream); `bpr_target` is seeded live from each sample. Documented in Task 4 + README.

**Placeholder scan:** none — every step has complete, runnable code or an exact command + expected result.

**Type consistency:** `TelemetrySample`/`SettleEvent`/`ParsedLine` (Task 2) are used unchanged in `format`, `app`, `store`, `index`, replay. `TuningView`/`KeyResult` (Task 4) are consumed by `app` (Task 6). `SerialStream`/`SerialLinkOptions` (Task 5) are implemented by the fake (Task 5 test) and the real `SerialPort` (Task 8). `MongoStore` method names (`connect/insertTelemetry/insertSettle/relabel/close`, Task 7) match every call site in `index` (Task 8) and the replay test (Task 9).

## Out of scope (YAGNI, per spec)
Web server, sockets, auth, multi-device, in-TUI history/sparklines, config files, JSON string-escaping beyond controlled identifiers. Analysis happens in Mongo (queries / Compass / the mongodb MCP), not the TUI.
