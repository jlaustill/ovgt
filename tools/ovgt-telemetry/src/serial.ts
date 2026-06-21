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
