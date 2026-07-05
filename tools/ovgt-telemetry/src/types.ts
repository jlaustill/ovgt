export interface TelemetrySample {
  type: "t";
  t_ms: number;
  mode: "auto" | "manual" | "brake";
  cop_hpa: number;
  cip_hpa: number;
  boost_psi: number;
  br: number;
  tip_psi: number;
  bpr: number;
  bpr_target: number;
  boost_region: "spool" | "pi";
  boost_integ: number;
  cit_c: number;
  cot_c: number;
  tit_c: number;
  mcu_c: number;
  ce_pct: number;
  ce_settled: boolean;
  dem_pct: number;
  pos_pct: number;
  act_load?: number; // actuator motor effort (0 = coasting, ~2220 = straining). Optional: absent in pre-2026-07-05 logs.
  act_temp?: number; // actuator body temp (raw feedback byte). Optional: absent in pre-2026-07-05 logs.
  brake: boolean;
}

export interface SettleEvent {
  type: "s";
  t_ms: number;
  tau_s: number;
  settle_s: number;
  step_c: number;
  cot_slope_c_s: number;
  boost_slope_psi_s: number;
  settle_timer_s: number;
}

// 1 Hz J1939 diagnostic line: online flags + per-signal health (h_<signal>).
export interface J1939DiagDoc {
  type: "d";
  t_ms: number;
  engine_online: boolean;
  trans_online: boolean;
  engine_up_ms: number;
  trans_up_ms: number;
  // h_<signal> health keys ("ok"|"na"|"err"|"absent"|"waiting") + unk_n/unk_dropped
  [key: string]: unknown;
}

// One line per undecoded PGN discovered on the bus.
export interface J1939UnknownDoc {
  type: "u";
  t_ms: number;
  pgn: number;
  sa: number;
  cnt: number;
  hz: number;
  last: string;
}

export type ParsedLine =
  | { kind: "telemetry"; sample: TelemetrySample }
  | { kind: "settle"; event: SettleEvent }
  | { kind: "j1939diag"; doc: J1939DiagDoc }
  | { kind: "j1939unknown"; doc: J1939UnknownDoc }
  | { kind: "log"; text: string };
