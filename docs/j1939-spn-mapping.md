# OVGT Telemetry → Standard J1939 SPN/PGN Mapping

Goal: transmit every OVGT signal on its **standard** J1939 SPN/PGN where one exists,
so off-the-shelf J1939 tooling decodes it with no custom DBC. Only signals with no
standard equivalent stay proprietary. Sourced from the local `j1939.pgns` database
(J1939-71 unless noted).

## Already transmitting (correct)

| Signal | SPN | PGN | Resolution | Note |
|---|---|---|---|---|
| Engine oil temp | 175 | 65262 | 0.03125 °C/bit | ✅ |
| Turbo oil temp | 176 | 65262 | 0.03125 °C/bit | ✅ |
| Oil pressure | 100 | 65263 | 4 kPa/bit | ✅ |
| Fuel / lift-pump pressure | 94 | 65263 | 4 kPa/bit | ✅ (Fuel Delivery Pressure) |

## Already transmitting, but a better SPN exists (resolution / specificity upgrade)

| Signal | Using | Better | PGN | Why |
|---|---|---|---|---|
| Boost (COP−CIP) | 102 Intake Manifold #1 Pressure (2 kPa/bit) | **1127 Turbocharger 1 Boost Pressure** (0.125 kPa/bit) | 65190 | Turbo-specific, 16× finer |
| TIT (turbine inlet / EGT) | 173 Exhaust Gas Temperature | **1180 Turbocharger 1 Turbine Intake Temperature** (0.03125 °C/bit) | 65176 | Exact "turbine inlet temp" match |

(102 and 173 are still valid generic signals — can transmit both the generic and the
turbo-specific if bus budget allows.)

## Have the data, NOT yet transmitting — add these standard SPNs

| Signal | SPN | PGN | Resolution | Note |
|---|---|---|---|---|
| **CIT** compressor inlet temp | **1172** Turbocharger 1 Compressor Intake Temperature | 65178 | 0.03125 °C/bit | exact — the heat-soak signal |
| **COT** compressor outlet temp | **2629** Turbocharger 1 Compressor Outlet Temperature | 64979 | 0.03125 °C/bit | exact |
| **CIP** compressor inlet pressure | **1176** Turbocharger 1 Compressor Intake Pressure | 65177 | 1/128 kPa/bit | exact |
| **TIP** turbine inlet / drive pressure | **1209** Engine Exhaust Gas Pressure | 65170 | 1/128 kPa/bit | best standard fit (exhaust pressure at turbine) |

## COP (compressor outlet pressure) — no exact SPN

No turbocharger compressor **outlet** pressure SPN exists. Options:
- Publish **CIP (1176) + Boost (1127)**; COP = CIP + boost is derivable downstream. **(recommended)**
- Or approximate with **3563 / 4817 Intake Manifold #1 Absolute Pressure** (PGN 64976) —
  but that's post-intercooler, a different measurement point.

## Vane position — only imperfect standard SPNs

No standard **variable-geometry** vane SPN. The nearest are wastegate-actuator SPNs
(ISOBUS-tagged, semantically a wastegate, not VGT vanes — resolution unspecified):
- Demanded (`dem_pct`) → **5370** Turbocharger Wastegate Actuator 1 **Desired** Position (PGN 65174)
- Reported (`pos_pct`) → **1188** Turbocharger Wastegate Actuator 1 Position (PGN 65174)

Judgment call: borrow these (loose fit, decodes on ISOBUS-aware tools) or keep vane
position proprietary. Leaning proprietary since a VGT vane ≠ a wastegate.

## No standard SPN — keep proprietary

| Signal | Why |
|---|---|
| **CE** compressor efficiency | computed metric, not a J1939 parameter |
| **BPR** drive/boost ratio | computed metric |
| **MCU temp** (Teensy die) | not a vehicle parameter |

## Already receiving (sniffed) — correct

| Signal | SPN | PGN |
|---|---|---|
| Accelerator pedal | 91 | 61443 (EEC2) |
| Engine percent load | 92 | 61443 (EEC2) |
| Torque converter lockup | 573 | 61442 (ETC1) |

## Bonus (standard SPN exists if the sensor is added later)

| Signal | SPN | PGN | Resolution |
|---|---|---|---|
| Turbocharger speed | 103 | 65245 | 4 rpm/bit |

## Net change to firmware transmit list

Add PGNs: **65176** (TIT 1180), **65178** (CIT 1172), **64979** (COT 2629),
**65177** (CIP 1176), **65170** (TIP 1209), **65190** (boost 1127).
Keep proprietary: CE, BPR, MCU temp, (and vane position, pending decision).
