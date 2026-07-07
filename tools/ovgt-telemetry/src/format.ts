import type { TelemetrySample } from "./types";

export function formatTelemetry(s: TelemetrySample): string[] {
  const ce = s.ce_pct < 0 ? "--" : `${s.ce_pct.toFixed(0)}%${s.ce_settled ? "" : "~"}`;
  // OVGT's own ADS2 oil sensors (firmware already converts pressure to psi).
  // Absent in pre-2026-07-07 logs -> show -- rather than a bogus 0.
  const oilTemp = s.oil_temp_c == null ? "--" : `${s.oil_temp_c.toFixed(0)}C`;
  const oilPress = s.oil_press_psi == null ? "--" : `${s.oil_press_psi.toFixed(0)}psi`;
  return [
    `mode ${s.mode}   brake ${s.brake ? "ON" : "off"}`,
    `boost ${s.boost_psi.toFixed(1)}psi  BR ${s.br.toFixed(2)}  TIP ${s.tip_psi.toFixed(1)}psi`,
    `BPR ${s.bpr.toFixed(2)}/${s.bpr_target.toFixed(2)}   dem ${s.dem_pct}%  pos ${s.pos_pct}%  load ${s.act_load ?? "--"}  aTemp ${s.act_temp ?? "--"}  cap ${s.vane_cap ?? "--"}`,
    `CIT ${s.cit_c.toFixed(1)}C  COT ${s.cot_c.toFixed(1)}C  TIT ${s.tit_c}C  MCU ${s.mcu_c.toFixed(1)}C`,
    `CE ${ce}   oil ${oilTemp} ${oilPress}`,
  ];
}
