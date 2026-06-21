import { render } from "ink-testing-library";
import React from "react";
import { expect, test } from "vitest";
import { App } from "./app";
import type { TelemetrySample } from "./types";

const sample: TelemetrySample = {
  type: "t", t_ms: 1, mode: "auto", cop_hpa: 871, cip_hpa: 875,
  boost_psi: 0, br: 1, tip_psi: 0, bpr: 0, bpr_target: 1,
  cit_c: 20.3, cot_c: 20.9, tit_c: 20, ce_pct: -1, ce_settled: false,
  dem_pct: 25, pos_pct: 27, brake: false,
};

const tick = () => new Promise((r) => setTimeout(r, 20));

// useInput subscribes in an effect (after render) and re-subscribes each
// render, so press() awaits a tick so the next handler closure is current.
async function press(stdin: { write: (s: string) => void }, s: string): Promise<void> {
  stdin.write(s);
  await tick();
}

test("renders the telemetry panel", () => {
  const { lastFrame } = render(
    <App sample={sample} logs={["Setup complete"]} sessionLabel="drive-1" status="open" onCommand={() => {}} onRelabel={() => {}} />,
  );
  expect(lastFrame()).toContain("COT 20.9C");
  expect(lastFrame()).toContain("drive-1");
  expect(lastFrame()).toContain("Setup complete");
});

test("pressing ] sends an absolute bpr command", async () => {
  const cmds: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={(c) => cmds.push(c)} onRelabel={() => {}} />,
  );
  await tick();
  await press(stdin, "]");
  expect(cmds).toContain("bpr 1.05");
});

test("digits then Enter send a clamped vane %", async () => {
  const cmds: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={(c) => cmds.push(c)} onRelabel={() => {}} />,
  );
  await tick();
  await press(stdin, "4");
  await press(stdin, "5");
  await press(stdin, "\r");
  expect(cmds).toContain("45");
});

test("l then typing then Enter relabels", async () => {
  const labels: string[] = [];
  const { stdin } = render(
    <App sample={sample} logs={[]} sessionLabel="t" status="open" onCommand={() => {}} onRelabel={(l) => labels.push(l)} />,
  );
  await tick();
  await press(stdin, "l");
  await press(stdin, "h");
  await press(stdin, "i");
  await press(stdin, "\r");
  expect(labels).toEqual(["hi"]);
});
