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
  ce_pct: number;
  ce_settled: boolean;
  dem_pct: number;
  pos_pct: number;
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

export type ParsedLine =
  | { kind: "telemetry"; sample: TelemetrySample }
  | { kind: "settle"; event: SettleEvent }
  | { kind: "log"; text: string };
