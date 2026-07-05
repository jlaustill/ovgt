import type { TelemetrySample } from "./types";

export function formatTelemetry(s: TelemetrySample): string[] {
  const ce = s.ce_pct < 0 ? "--" : `${s.ce_pct.toFixed(0)}%${s.ce_settled ? "" : "~"}`;
  return [
    `mode ${s.mode}   brake ${s.brake ? "ON" : "off"}`,
    `boost ${s.boost_psi.toFixed(1)}psi  BR ${s.br.toFixed(2)}  TIP ${s.tip_psi.toFixed(1)}psi`,
    `BPR ${s.bpr.toFixed(2)}/${s.bpr_target.toFixed(2)}   dem ${s.dem_pct}%  pos ${s.pos_pct}%  load ${s.act_load ?? "--"}  aTemp ${s.act_temp ?? "--"}`,
    `CIT ${s.cit_c.toFixed(1)}C  COT ${s.cot_c.toFixed(1)}C  TIT ${s.tit_c}C  MCU ${s.mcu_c.toFixed(1)}C`,
    `CE ${ce}`,
  ];
}
