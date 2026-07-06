import { EventEmitter } from "node:events";
import { readFileSync } from "node:fs";
import { hostname } from "node:os";
import { render } from "ink";
import React from "react";
import { SerialPort } from "serialport";
import { App } from "./app";
import { parseLine } from "./parse";
import { pickTeensyPort, teensyByIdPath, SerialLink, type SerialStream } from "./serial";
import { MongoStore } from "./store";
import type { SettleEvent, TelemetrySample } from "./types";

// OVGT turbo controller USB serial (RUNNING-firmware serial — a Teensy reports a
// DIFFERENT serial in HalfKay bootloader). Override with --serial or OVGT_SERIAL.
// vulCAN logger is 16550620; never connect to it by accident.
const OVGT_TEENSY_SERIAL = process.env.OVGT_SERIAL ?? "11969470";

interface Args {
  port?: string;
  serial?: string;
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
    else if (a === "--serial") args.serial = argv[++i];
    else if (a === "--mongo") args.mongo = argv[++i]!;
    else if (a === "--no-mongo") args.noMongo = true;
    else if (a === "--label") args.label = argv[++i]!;
    else if (a === "--replay") args.replay = argv[++i];
  }
  return args;
}

type Feed = EventEmitter;

function Root({ feed, onRelabel, initialLabel }: {
  feed: Feed;
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
    return () => {
      feed.removeAllListeners();
    };
  }, [feed]);

  return (
    <App
      sample={sample}
      settle={settle}
      logs={logs}
      sessionLabel={label}
      status={status}
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
      } else if (parsed.kind === "j1939diag") {
        await store?.insertJ1939Diag(parsed.doc);
      } else if (parsed.kind === "j1939unknown") {
        await store?.insertJ1939Unknown(parsed.doc);
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

  // Surface Mongo write health in the log (edge-triggered inside the store, so
  // an outage logs once, not once per 10 Hz sample).
  if (store) store.onWriteError = (message) => feed.emit("log", message);

  const handleLine = (line: string): void => {
    const parsed = parseLine(line);
    if (parsed.kind === "telemetry") {
      feed.emit("sample", parsed.sample);
      store?.recordTelemetry(parsed.sample);
    } else if (parsed.kind === "settle") {
      feed.emit("settle", parsed.event);
      store?.recordSettle(parsed.event);
    } else if (parsed.kind === "j1939diag") {
      store?.recordJ1939Diag(parsed.doc);
    } else if (parsed.kind === "j1939unknown") {
      store?.recordJ1939Unknown(parsed.doc);
    } else if (parsed.text !== "") {
      feed.emit("log", parsed.text);
    }
  };

  const serial = args.serial ?? OVGT_TEENSY_SERIAL;
  let portPath = args.port;
  let boardFound = false;
  if (!portPath) {
    // Resolve by serial. If present now → its stable by-id symlink. If absent →
    // STILL the by-id symlink (constructed), so open() keeps retrying THIS board
    // instead of ever falling back to a random /dev/ttyACM* (e.g. vulCAN).
    const found = pickTeensyPort(await SerialPort.list(), serial);
    boardFound = found !== undefined;
    portPath = found ?? teensyByIdPath(serial);
  }
  const link = new SerialLink({
    open: () => new SerialPort({ path: portPath!, baudRate: 115200 }) as unknown as SerialStream,
    onLine: handleLine,
    onStatus: (st, d) => feed.emit("status", d ? `${st}: ${d}` : st),
  });

  const instance = render(
    <Root
      feed={feed}
      onRelabel={(l) => void store?.relabel(l)}
      initialLabel={args.label}
    />,
  );

  // Start the source AFTER mount so Root's feed subscription is already live.
  setImmediate(() => {
    if (!args.port) {
      feed.emit("log", boardFound
        ? `OVGT board serial ${serial} → ${portPath}`
        : `waiting for OVGT board serial ${serial} (${portPath}); refusing any other device`);
    }
    if (store) feed.emit("log", `mongo: logging to ${args.mongo} db=ovgt`);
    link.start();
  });

  // 'q' (App's exit()) or Ctrl-C (Ink's default) resolve this; only then
  // release the serial port + Mongo so the process actually exits.
  await instance.waitUntilExit();
  link.stop();
  await store?.close();
  process.exit(0);
}

void main();
