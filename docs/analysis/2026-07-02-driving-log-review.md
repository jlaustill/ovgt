# Driving-log review — 2026-07-02

Analysis of ~710k telemetry samples (10 Hz) in `ovgt.telemetry`, spanning 5 sessions
over ~17 hours of 2026-07-02 driving (transmission-tuning day). All queries run against
local MongoDB `127.0.0.1:27017`.

## Sessions

| Session start | Label | Samples | Boost max | MCU avg/max | EGT avg/max |
|---|---|---|---|---|---|
| 12:51 | New Tranmission Tuning | 167k | 27.2 psi | 70 / 80 °C | 264 / 787 °C |
| 20:14 | (idle/parked) | 17k | 0.4 psi | 71 / 79 °C | 186 / 209 °C |
| 20:44 | New tranny tuning take 2 | 28k | 27.9 psi | 72 / 81 °C | 498 / 763 °C |
| 23:19 | (overnight) | 139k | 29.2 psi | 67 / 82 °C | 371 / 818 °C |
| 05:54 (07-03) | (loaded pull) | — | — | — | 640 / 831 °C |

## 1. Reboots — electrical/brown-out, not thermal

Detected via `t_ms` (firmware uptime) dropping between consecutive in-session samples.
**3 genuine reboot events**, each preceded by long stable uptime:

| Time | Uptime before | Load | MCU temp | Pattern |
|---|---|---|---|---|
| 18:17:46 | ~6.1 h | idle (0.13 psi) | 75 °C | double-burst (~18:17:49 again) |
| 19:27:48 | ~1.0 h | idle (0.12 psi) | 57.5 °C | double-burst (~19:27:50 again) |
| 03:53:51 | ~5.6 h | light (2.1 psi) | 56 °C | single |

Conclusions:
- **Not thermal.** Two of three reboots were at 56–57 °C — cooler than the all-day MCU
  average (66–72 °C) the board survives fine. Reboots do not cluster at the 80–82 °C peaks.
- **Double-burst signature.** Board resets, runs ~1.5 s, resets again, then stabilizes
  (uptime 839→2297 ms, drop to 1213 ms, climb again). Classic voltage brown-out or a
  watchdog firing before init completes. USB CDC drops ~2.2 s per reset.
- **Idle/coast only.** All reboots at <2.1 psi. 27–29 psi pulls all day caused none.
  A transmission-tuning day → shift-solenoid / alternator load transient during coast is a
  plausible rail-sag mechanism.
- **Boot actuator transient:** on every reboot `pos_pct` snaps to ~79–80% open, then the
  controller pulls it to idle (~23%) within ~300 ms. Harmless at idle but uncommanded.

Two additional sub-second `t_ms` back-steps (23:54: −104 ms; 00:00: −208 ms) are **not**
reboots (uptime stayed ~5.7 M ms). Both at high MCU temp (78–82 °C) under load — likely
telemetry timestamp/scheduling jitter. Distinct from the reboot issue; revisit when
loop-timing profiling (`systemHealth`) lands.

**Recommended next instrumentation:** capture the Teensy reset-cause register (POR /
brown-out / watchdog) on next drive to confirm the mechanism — a rail-voltage/brown-out
flag will discriminate faster than more thermal logging.

## 2. Compressor efficiency (CE) — trustworthy only at steady state

Of 343k auto-mode samples: 25% are the `−1` "can't compute" sentinel, 9% are non-physical
spikes (CE > 100%, peaking at 3466%), only 66% are physical 0–100 values.

Settle fraction is inverted relative to need: **spool region 87% settled, PI (on-boost)
region 21% settled.**

Bucketing PI-region samples by |Δboost| over ~0.5 s:

| |Δboost| / 0.5s | samples | CE settled | CE physical | avg boost |
|---|---|---|---|---|---|
| < 0.5 psi (steady) | 91,504 | 22.8% | 96% | 5.4 |
| 0.5–1 | 5,095 | 0% | 74% | 10.1 |
| 1–2 | 2,974 | 0% | 64% | 12.3 |
| 2–4 | 751 | 0% | — | 14.4 |
| > 4 | 145 | 0% | 91% | 12.5 |

**The instant boost moves faster than ~0.5 psi/0.5 s, CE settled-fraction is exactly 0.**
Filtering to settled + physical + boost ≥ 8 psi (a reading usable for control during a pull)
yields **312 samples in the whole day**. When settled on boost, CE is tight and believable
(median 59%, p10–p90 = 56–64%) — the problem is *availability*, not accuracy.

This is the quantified case for Phase 2 (faster COT probe): the settle gate is correct;
the probe's 5–6 s thermal lag makes "settled + on boost" a near-empty set. Software
time-alignment (Phase 1) cannot recover a signal that is never produced during transients.

## 3. Boost control (BPR tracking)

BPR target = **1.5** all day (NOTE: `project_bpr_boost_control` memory says target=1.0 —
stale). PI region: BPR averaged 2.60 vs 1.5 target, mean abs error 1.24 — consistently
**above** target (more backpressure / more closed than commanded). Actuator tracks demand
tightly (pos ≈ dem within ~1% avg in both regions), so the gap is control-authority/tuning,
not actuator tracking.

## 4. EGT / spool-floor check

Compared spool-region `pos_pct` between the 12:51 and 20:44 sessions: **both held
pos ≈ 22.6 (median 22)** — vanes were at floor **22** in both. The 20:44 session's higher
EGT (498 vs 264 °C avg) is **driving intensity, not a lower vane floor**. The spool=18 EGT
spike that prompted the revert is not present in this driving data — the flashed firmware
ran floor 22 throughout. (The `22→18` commit is newest but was not executed by these drives,
or was reverted before them.)

## 5. Why the same backpressure gives wildly different boost — it's exhaust energy (EGT)

Question: sometimes 10 psi backpressure (TIP) → ~10 psi boost, other times → almost none.

Held TIP at 9–11 psi, **steady state only** (|Δboost| < 0.5 psi over 1 s), bucketed by EGT:

| EGT band | n | boost | vane pos% | BPR |
|---|---|---|---|---|
| < 300 °C | 1000 | 0.77 psi | 22.2 (floor) | 18.8 |
| 300–400 | 1254 | 1.76 | 22.1 (floor) | 7.1 |
| 400–500 | 1117 | 2.88 | 22.1 (floor) | 3.8 |
| 500–600 | 10114 | 4.39 | 22.3 (floor) | 2.7 |
| 600–700 | 5520 | 5.66 | 24.8 | 1.9 |
| > 700 | 1944 | 7.65 | 28.1 | 1.3 |

At fixed 10 psi backpressure, boost scales **10× (0.77 → 7.65 psi) purely with EGT**, with the
vane pinned at the ~22% floor for the bottom four buckets. Backpressure only sets the turbine
pressure-ratio term; turbine power ≈ `ṁ · cp · T_in · (1 − PR^−0.28)`, so at fixed PR the boost
is governed by exhaust energy `ṁ · T_in`, which rises with fueling/load. Cold low-flow exhaust
(light throttle / overrun) makes backpressure by restriction with no energy to spin the turbine
→ BPR ~19, no boost. Hot high-flow exhaust (heavy fueling) → BPR ~1.3, backpressure converts to
boost.

Rules out turbo lag: this is steady-state, and lag would show hot EGT with boost still climbing,
not the cold-EGT low-boost cases seen here.

**Controls implication:** below an exhaust-energy floor, no vane position can make boost, yet the
BPR=1.5 controller sits at the spool floor commanding unachievable backpressure. Feed-forward/target
should be scheduled on exhaust energy / operating point (EGT or load), not backpressure alone.

**Missing variable:** no engine RPM / MAF is logged, so energy can't be split into mass-flow vs
temperature. RPM is on J1939 (SPN 190) via the existing OSSM path — logging it would enable a real
turbine-power / mass-flow map (Phase 3 feed-forward).

## Key queries

- Reboot detection: `$setWindowFields` partition by `sessionId`, `$shift` prev `t_ms`,
  match `t_ms < prevTms`.
- CE quality: `$group` counting `ce_pct == -1`, `> 100`, and `0..100`.
- CE-vs-transient: PI region, `$shift` boost by −5 samples, `$bucket` on |Δboost|,
  avg `ce_settled`.
