import { EventEmitter } from "node:events";
import { PassThrough } from "node:stream";
import { expect, test, vi } from "vitest";
import { pickTeensyPort, teensyByIdPath, SerialLink, type SerialStream } from "./serial";

test("pickTeensyPort finds the sole 16c0 device, case-insensitive", () => {
  const ports = [
    { path: "/dev/ttyS0", vendorId: undefined },
    { path: "/dev/ttyACM0", vendorId: "16C0", productId: "0483" },
  ];
  expect(pickTeensyPort(ports)).toBe("/dev/ttyACM0"); // one Teensy, no serial → its path
  expect(pickTeensyPort([{ path: "/dev/ttyS0" }])).toBeUndefined();
});

test("pickTeensyPort selects the RIGHT board by serial when several are present", () => {
  const ports = [
    { path: "/dev/ttyACM1", vendorId: "16c0", productId: "0483", serialNumber: "16550620" }, // vulCAN
    { path: "/dev/ttyACM2", vendorId: "16c0", productId: "0483", serialNumber: "11969470" }, // OVGT
  ];
  // Targets OVGT by serial → its stable by-id symlink, NOT vulCAN's ttyACM1.
  expect(pickTeensyPort(ports, "11969470")).toBe(teensyByIdPath("11969470"));
  expect(pickTeensyPort(ports, "16550620")).toBe(teensyByIdPath("16550620"));
});

test("pickTeensyPort fails CLOSED: target absent, or ambiguous with no target", () => {
  const ports = [
    { path: "/dev/ttyACM1", vendorId: "16c0", serialNumber: "16550620" },
    { path: "/dev/ttyACM2", vendorId: "16c0", serialNumber: "11969470" },
  ];
  expect(pickTeensyPort(ports, "99999999")).toBeUndefined(); // requested board not plugged in
  expect(pickTeensyPort(ports)).toBeUndefined(); // 2 Teensies, no target → refuse to guess
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

// USB re-enumeration (vibration drop) makes open() throw ENOENT for a moment;
// the link must keep retrying instead of dying on the error.
test("retries when open() throws (device briefly absent)", async () => {
  let calls = 0;
  const open = vi.fn(() => {
    calls++;
    if (calls === 1) throw new Error("No such file or directory, cannot open /dev/ttyACM0");
    return new FakeSerial();
  });
  const link = new SerialLink({ open, onLine: () => {}, reconnectMs: 5 });
  link.start();
  await new Promise((r) => setTimeout(r, 20));
  expect(open).toHaveBeenCalledTimes(2);
});

// A mid-stream disconnect can surface as an "error" event (not "close"); it must
// still trigger a reconnect.
test("reconnects after an error event", async () => {
  const open = vi.fn(() => new FakeSerial());
  const link = new SerialLink({ open, onLine: () => {}, reconnectMs: 5 });
  link.start();
  (open.mock.results[0]!.value as FakeSerial).emit("error", new Error("disconnected"));
  await new Promise((r) => setTimeout(r, 20));
  expect(open).toHaveBeenCalledTimes(2);
});

// The lockup bug: bytes silently stop arriving with NO "close" and NO "error"
// event (Teensy still powered, cable still seated, USB CDC just stalled). Without
// a watchdog the link waits forever; only a physical replug revives it. The link
// must detect the silence and force a reconnect on its own.
test("reconnects after a silent stall (no data, no close/error)", async () => {
  const open = vi.fn(() => new FakeSerial());
  const link = new SerialLink({ open, onLine: () => {}, reconnectMs: 5, stallTimeoutMs: 15 });
  link.start();
  const first = open.mock.results[0]!.value as FakeSerial;
  first.inject('{"type":"t"}\n'); // one line arrives, then the stream goes quiet
  await new Promise((r) => setTimeout(r, 40)); // silence longer than stallTimeoutMs
  expect(open).toHaveBeenCalledTimes(2);
});

// The watchdog must not fire while telemetry is flowing normally: each received
// line resets the inactivity clock.
test("does not reconnect while data keeps arriving", async () => {
  const open = vi.fn(() => new FakeSerial());
  const link = new SerialLink({ open, onLine: () => {}, reconnectMs: 5, stallTimeoutMs: 30 });
  link.start();
  const port = open.mock.results[0]!.value as FakeSerial;
  for (let i = 0; i < 5; i++) {
    port.inject('{"type":"t"}\n');
    await new Promise((r) => setTimeout(r, 10)); // 10ms gap < 30ms stall timeout
  }
  expect(open).toHaveBeenCalledTimes(1);
});
