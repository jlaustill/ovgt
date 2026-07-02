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
  private writeFailing = false;

  // Notified when live-write health changes. Edge-triggered: fires once when
  // writes start failing and once when they recover, so a Mongo outage cannot
  // flood the UI with one message per 10 Hz sample.
  onWriteError?: (message: string) => void;

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

  // Fire-and-forget writes for the live path. These never reject — a failed
  // write is reported through onWriteError on transition, not per sample — so a
  // Mongo outage neither crashes the process (unhandled rejection) nor blocks
  // the serial read/render loop. Replay uses the awaited insert* methods above.
  recordTelemetry(sample: TelemetrySample): void {
    this.guardWrite(this.insertTelemetry(sample));
  }

  recordSettle(event: SettleEvent): void {
    this.guardWrite(this.insertSettle(event));
  }

  private guardWrite(write: Promise<void>): void {
    write.then(
      () => {
        if (this.writeFailing) {
          this.writeFailing = false;
          this.onWriteError?.("mongo writes recovered");
        }
      },
      (err: unknown) => {
        if (!this.writeFailing) {
          this.writeFailing = true;
          this.onWriteError?.(`mongo write failed: ${(err as Error).message}`);
        }
      },
    );
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
