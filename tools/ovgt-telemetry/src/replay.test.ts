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
  expect(logs).toBe(2); // "Setup complete" + "BPR target = 1.00"
});
