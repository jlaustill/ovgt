import { expect, test } from "vitest";
import { formatTelemetry } from "./format";
import type { TelemetrySample } from "./types";

const sample: TelemetrySample = {
  type: "t", t_ms: 46649, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  boost_region: "spool", boost_integ: 0,
  cit_c: 20.32, cot_c: 20.91, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};

test("formats core telemetry fields", () => {
  const lines = formatTelemetry(sample).join("\n");
  expect(lines).toContain("mode auto");
  expect(lines).toContain("COT 20.9C");
  expect(lines).toContain("BPR 0.00/1.00");
  expect(lines).toContain("dem 25%");
});

test("ce_pct -1 renders as -- and settled drops the ~", () => {
  expect(formatTelemetry(sample).join("\n")).toContain("CE --");
  const warm = { ...sample, ce_pct: 71, ce_settled: false };
  expect(formatTelemetry(warm).join("\n")).toContain("CE 71%~");
  const settled = { ...warm, ce_settled: true };
  expect(formatTelemetry(settled).join("\n")).toContain("CE 71%");
  expect(formatTelemetry(settled).join("\n")).not.toContain("71%~");
});
