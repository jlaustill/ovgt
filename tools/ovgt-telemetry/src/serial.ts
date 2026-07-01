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
  private reconnectTimer?: ReturnType<typeof setTimeout>;

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
    parser.on("data", (line: string) => this.opts.onLine(line));
    // A disconnect can surface as either "error" or "close" (or both); either
    // must schedule a reconnect. scheduleReconnect() de-dupes.
    port.on("error", (err) => {
      this.opts.onStatus?.("error", err.message);
      this.scheduleReconnect();
    });
    port.on("close", () => {
      this.opts.onStatus?.("closed");
      this.scheduleReconnect();
    });
    this.opts.onStatus?.("open");
  }

  private scheduleReconnect(): void {
    if (this.stopped || this.reconnectTimer) return;
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
  }
}
