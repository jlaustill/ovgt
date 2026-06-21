import { expect, test } from "vitest";
import { parseLine } from "./parse";

const telemetry =
  '{"type":"t","t_ms":46649,"mode":"auto","cop_hpa":871,"cip_hpa":875,"boost_psi":0.00,"br":1.00,"tip_psi":0.00,"bpr":0.00,"bpr_target":1.00,"cit_c":20.32,"cot_c":20.91,"tit_c":20,"ce_pct":-1.00,"ce_settled":false,"dem_pct":25,"pos_pct":27,"brake":false}';
const settle =
  '{"type":"s","t_ms":99999,"tau_s":5.20,"settle_s":8.10,"step_c":42.30,"cot_slope_c_s":0.05,"boost_slope_psi_s":0.01,"settle_timer_s":2.30}';

test("parses a telemetry line", () => {
  const r = parseLine(telemetry);
  expect(r.kind).toBe("telemetry");
  if (r.kind === "telemetry") {
    expect(r.sample.t_ms).toBe(46649);
    expect(r.sample.mode).toBe("auto");
    expect(r.sample.cot_c).toBeCloseTo(20.91);
    expect(r.sample.ce_settled).toBe(false);
  }
});

test("parses a settle line", () => {
  const r = parseLine(settle);
  expect(r.kind).toBe("settle");
  if (r.kind === "settle") {
    expect(r.event.tau_s).toBeCloseTo(5.2);
    expect(r.event.settle_timer_s).toBeCloseTo(2.3);
  }
});

test("non-JSON banner becomes a log line", () => {
  const r = parseLine("Setup complete");
  expect(r).toEqual({ kind: "log", text: "Setup complete" });
});

test("JSON without a known type becomes a log line", () => {
  const r = parseLine('{"foo":1}');
  expect(r.kind).toBe("log");
});

test("blank line becomes an empty log line", () => {
  expect(parseLine("  \r")).toEqual({ kind: "log", text: "" });
});
