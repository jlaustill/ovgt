import { MongoClient } from "mongodb";
import { afterAll, expect, test, vi } from "vitest";
import { MongoStore, type SessionDoc } from "./store";
import type { SettleEvent, TelemetrySample } from "./types";

const uri = "mongodb://127.0.0.1:27017";
const dbName = `ovgt_test_${Date.now()}`;

const sample: TelemetrySample = {
  type: "t", t_ms: 100, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  boost_region: "spool", boost_integ: 0, mcu_c: 45.3,
  cit_c: 20.3, cot_c: 20.9, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};
const settle: SettleEvent = {
  type: "s", t_ms: 200, tau_s: 5.2, settle_s: 8.1, step_c: 42.3,
  cot_slope_c_s: 0.05, boost_slope_psi_s: 0.01, settle_timer_s: 2.3,
};
const diag = {
  type: "d" as const, t_ms: 300, engine_online: true, trans_online: false,
  engine_up_ms: 4200, trans_up_ms: 0, h_engine_rpm: "ok", h_system_v: "absent",
};
const unknown = {
  type: "u" as const, t_ms: 400, pgn: 65247, sa: 0, cnt: 5, hz: 20, last: "FFFF03FFFFFFFFFF",
};

afterAll(async () => {
  const client = new MongoClient(uri);
  await client.connect();
  await client.db(dbName).dropDatabase();
  await client.close();
});

// No Mongo needed: the store is never connected; we stub the insert to control
// success/failure and assert the fire-and-forget path reports edge-triggered.
test("live writes report Mongo failures once per outage, and recovery", async () => {
  const store = new MongoStore(uri);
  const messages: string[] = [];
  store.onWriteError = (m) => messages.push(m);
  const flush = () => new Promise((r) => setTimeout(r, 0));

  const insert = vi.spyOn(store, "insertTelemetry").mockRejectedValue(new Error("ECONNREFUSED"));
  store.recordTelemetry(sample);
  store.recordTelemetry(sample);
  store.recordTelemetry(sample);
  await flush();
  // 3 failed writes, but only ONE message — no flood at 10 Hz.
  expect(messages).toEqual(["mongo write failed: ECONNREFUSED"]);

  insert.mockResolvedValue(undefined);
  store.recordTelemetry(sample);
  await flush();
  expect(messages).toEqual(["mongo write failed: ECONNREFUSED", "mongo writes recovered"]);
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
  const sess = await db.collection<SessionDoc>("sessions").findOne({ _id: sessionId });
  await client.close();

  expect(tdoc?.cot_c).toBeCloseTo(20.9);
  expect(tdoc?.ts).toBeInstanceOf(Date);
  expect(sdoc?.tau_s).toBeCloseTo(5.2);
  expect(sess?.label).toBe("relabeled");
  expect(sess?.endTs).toBeInstanceOf(Date);
});

test("stores j1939 diag + unknown docs with sessionId and ts", async () => {
  const store = new MongoStore(uri, dbName);
  const sessionId = await store.connect("j1939-drive", "testhost");
  await store.insertJ1939Diag(diag);
  await store.insertJ1939Unknown(unknown);
  await store.close();

  const client = new MongoClient(uri);
  await client.connect();
  const db = client.db(dbName);
  const ddoc = await db.collection("j1939_diag").findOne({ sessionId });
  const udoc = await db.collection("j1939_unknown").findOne({ sessionId });
  await client.close();

  expect(ddoc?.engine_online).toBe(true);
  expect(ddoc?.h_engine_rpm).toBe("ok");
  expect(ddoc?.ts).toBeInstanceOf(Date);
  expect(udoc?.pgn).toBe(65247);
  expect(udoc?.last).toBe("FFFF03FFFFFFFFFF");
});
