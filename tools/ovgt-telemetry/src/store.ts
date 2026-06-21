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
