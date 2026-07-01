import { MongoClient } from "mongodb";
import { afterAll, expect, test } from "vitest";
import { MongoStore, type SessionDoc } from "./store";
import type { SettleEvent, TelemetrySample } from "./types";

const uri = "mongodb://127.0.0.1:27017";
const dbName = `ovgt_test_${Date.now()}`;

const sample: TelemetrySample = {
  type: "t", t_ms: 100, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  boost_region: "spool", boost_integ: 0,
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
  const sess = await db.collection<SessionDoc>("sessions").findOne({ _id: sessionId });
  await client.close();

  expect(tdoc?.cot_c).toBeCloseTo(20.9);
  expect(tdoc?.ts).toBeInstanceOf(Date);
  expect(sdoc?.tau_s).toBeCloseTo(5.2);
  expect(sess?.label).toBe("relabeled");
  expect(sess?.endTs).toBeInstanceOf(Date);
});
