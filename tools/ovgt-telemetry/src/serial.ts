import { ReadlineParser } from "@serialport/parser-readline";

export interface PortLike {
  path: string;
  vendorId?: string;
  productId?: string;
  serialNumber?: string;
}

const TEENSY_VENDOR_ID = "16c0";

// The stable, serial-keyed symlink udev creates for a running Teensy. Unlike
// /dev/ttyACM*, it always points at THIS specific board and survives USB
// re-enumeration — so we open the board we mean, never "device number 1".
export function teensyByIdPath(serial: string): string {
  return `/dev/serial/by-id/usb-Teensyduino_USB_Serial_${serial}-if00`;
}

// Resolve the port for a specific Teensy by USB serial number. Matching by
// serial (not by /dev path, not by vendor alone) is what stops us grabbing the
// WRONG board when several 16c0 Teensies are plugged in — e.g. the OVGT turbo
// controller vs the vulCAN logger, which share vendor 16c0. Returns the stable
// by-id symlink for the match. Fails CLOSED: undefined when the requested board
// is absent (or when no target is given and several Teensies are present), so
// callers refuse to connect rather than talking to a random device.
export function pickTeensyPort(ports: PortLike[], targetSerial?: string): string | undefined {
  const teensies = ports.filter((p) => (p.vendorId ?? "").toLowerCase() === TEENSY_VENDOR_ID);
  const match = targetSerial
    ? teensies.find((p) => p.serialNumber === targetSerial)
    : teensies.length === 1
      ? teensies[0]
      : undefined;
  if (!match) return undefined;
  return match.serialNumber ? teensyByIdPath(match.serialNumber) : match.path;
}

// Minimal surface shared by a real SerialPort and the test's fake stream.
export interface SerialStream {
  pipe<T extends NodeJS.WritableStream>(dest: T): T;
  write(data: string): boolean;
  on(event: "close", cb: () => void): this;
  on(event: "error", cb: (err: Error) => void): this;
  // Real SerialPort exposes close(); releasing the OS file descriptor is what
  // forces cdc_acm to drop and re-open the device after a silent stall. Optional
  // so the test's fake stream (which has no fd) need not implement it.
  close?(cb?: (err?: Error | null) => void): void;
}

export interface SerialLinkOptions {
  open: () => SerialStream;
  onLine: (line: string) => void;
  onStatus?: (status: "open" | "closed" | "error" | "stalled", detail?: string) => void;
  reconnectMs?: number;
  // Force a reconnect if no line arrives for this long. Telemetry runs at 10 Hz
  // (~100ms/line), so a couple of seconds of total silence means the stream is
  // dead even though no error/close event fired.
  stallTimeoutMs?: number;
}

export class SerialLink {
  private port?: SerialStream;
  private stopped = false;
  private reconnectTimer?: ReturnType<typeof setTimeout>;
  private activityTimer?: ReturnType<typeof setTimeout>;

  constructor(private readonly opts: SerialLinkOptions) {}

  start(): void {
    this.stopped = false;
    this.connect();
  }

  private connect(): void {
    let port: SerialStream;
    try {
      port = this.opts.open();
    } catch (err) {
      // Device briefly absent — e.g. the Teensy's USB re-enumerating after a
      // vibration-induced disconnect. Report and keep retrying, don't die.
      this.opts.onStatus?.("error", (err as Error).message);
      this.scheduleReconnect();
      return;
    }
    this.port = port;
    const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
    parser.on("data", (line: string) => {
      if (port !== this.port) return; // ignore a torn-down port's stragglers
      this.noteActivity();
      this.opts.onLine(line);
    });
    // A disconnect can surface as either "error" or "close" (or both); either
    // must schedule a reconnect. scheduleReconnect() de-dupes. The identity guard
    // stops a stale (already-replaced) port from tearing down a fresh connection.
    port.on("error", (err) => {
      if (port !== this.port) return;
      this.opts.onStatus?.("error", err.message);
      this.scheduleReconnect();
    });
    port.on("close", () => {
      if (port !== this.port) return;
      this.opts.onStatus?.("closed");
      this.scheduleReconnect();
    });
    this.opts.onStatus?.("open");
    this.noteActivity(); // arm the stall watchdog for this fresh connection
  }

  // Called on every received line and on a fresh connect: (re)arm the inactivity
  // watchdog. If it fires, the stream has gone silent → handleStall().
  private noteActivity(): void {
    if (this.activityTimer) clearTimeout(this.activityTimer);
    if (this.stopped) return;
    this.activityTimer = setTimeout(() => this.handleStall(), this.opts.stallTimeoutMs ?? 2000);
  }

  // TODO(you): the stall recovery policy — you decided "2s silence → tear down
  // and reconnect" (mirrors what a physical replug does). Implement it here.
  //
  // When this fires, `this.port` is the stalled port and NO error/close event
  // ever came. You need to:
  //   1. Tell the UI something changed          -> this.opts.onStatus?.("stalled")
  //   2. Release the current port's fd so the OS can re-enumerate cdc_acm.
  //      The real port has .close() (optional on the interface); the old port's
  //      "close" event is guarded away once step 3 runs, so call it best-effort:
  //          this.port?.close?.();
  //   3. Detach from the stalled port so its late events are ignored, then
  //      reopen: clear this.port, then this.scheduleReconnect().
  //
  // Trade-off to weigh: is closing the fd (step 2) strictly required, or would
  // reopening a second handle be enough? On Linux cdc_acm a stuck fd usually must
  // be released before a fresh open() delivers bytes again — which is exactly why
  // only a physical replug fixes it today.
  private handleStall(): void {
    if (this.stopped || this.port === undefined) return;
    this.opts.onStatus?.("stalled");
    const stalled = this.port;
    this.port = undefined;   // detach first: the identity guards now ignore any
                             // late close/error/data from this stalled port
    stalled.close?.();       // best-effort fd release so cdc_acm can re-enumerate
    this.scheduleReconnect();
  }

  private scheduleReconnect(): void {
    if (this.stopped || this.reconnectTimer) return;
    if (this.activityTimer) clearTimeout(this.activityTimer);
    this.activityTimer = undefined;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = undefined;
      this.connect();
    }, this.opts.reconnectMs ?? 2000);
  }

  send(command: string): void {
    this.port?.write(command.endsWith("\n") ? command : `${command}\n`);
  }

  stop(): void {
    this.stopped = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = undefined;
    }
    if (this.activityTimer) {
      clearTimeout(this.activityTimer);
      this.activityTimer = undefined;
    }
  }
}
