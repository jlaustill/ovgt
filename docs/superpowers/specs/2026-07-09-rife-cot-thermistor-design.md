# RIFE Hi-AT thermistor replaces the COT thermocouple

**Date:** 2026-07-09
**Status:** Approved (design)
**Relates to:** CE real-time accuracy epic, Phase 2 (faster COT probe) —
`docs/plans/2026-06-22-realtime-ce-accuracy-epic.md`, `docs/north-star.md`

## Goal

Compressor-outlet temperature (COT) is the dominant lag term in the compressor-
efficiency (CE) calculation. The current COT source is a sheathed MAX31856 K-type
thermocouple with a measured τ≈5–6 s. Replacing it with a fast open-element RIFE
Hi-AT NTC thermistor cuts that lag, which is the whole point of Phase 2 — CE cannot
be controlled on until it is time-aligned enough to act on.

The thermistor reads through an ADS1115 analog channel. We reuse the never-hooked-up
"fuel/lift-pump pressure" input (ADS2 `0x49` channel 3) rather than adding hardware.

## Non-goals

- Re-tuning `cotSettle` thresholds for the faster probe. Its τ and settled-flag
  behavior will differ, but that needs real on-truck data and ties into the pending
  "settled flag never clears" fix. Out of scope; a follow-up.
- Changing the EMA filtering on the other seven ADC channels.
- Migrating this code to C-Next now (the extracted pure module is a natural future
  candidate).

## Sensor characterization

RIFE Hi-AT (`docs/Hi-AT-Air-Temperature-Sensor.pdf`), a 2-wire NTC with a 32-point
R/T table spanning 1.59 MΩ @ −10 °F down to 208 Ω @ 485 °F.

**Pull-up: 1.0 kΩ, 1 %, to 5 V; sensor to GND** (its own value — CIT and oil-temp
keep their 2.2 kΩ). Chosen for the 250–485 °F COT-under-load band (peak ~500 °F):
mean divider sensitivity is flat (~22 mV/°C) for any pull-up 680 Ω–1 kΩ, so the band
is insensitive to the exact value; 1.0 kΩ best centers the voltage swing
(3.77 V @ 250 °F → 0.86 V @ 485 °F, no railing) and is a standard value.

**Conversion: Steinhart-Hart**, `1/T(K) = A + B·lnR + C·lnR³`, fit to the full table:

    A = 6.535185e-4
    B = 2.345466e-4
    C = 9.380459e-11

Max fit error 0.11 °C over the entire −10…485 °F range (the datasheet cells are
"calculated," i.e. an idealized curve — no table lookup needed).

Divider inversion: `R = 1000 · V / (5 − V)`.

## Design

### 1. Extracted pure conversion module (`cotThermistor`)

New `src/sensors/cotThermistor.h/.cpp` — pure C++, no Arduino/ADS dependencies, so it
is unit-testable on the native host. Holds the pull-up constant, the RIFE Steinhart-
Hart coefficients, and the divider math.

    // Returns true and writes *outTempC when the sample is a valid reading;
    // returns false when the divider is railed (open/short) so the caller holds
    // the last good value.
    bool cotThermistorReadC(float voltage, float *outTempC);

Rail rejection: `voltage >= 4.99 || voltage <= 0.01` → return false. Cold COT
(<~150 °F, thermistor ~90 kΩ) sits near the 5 V rail and reads low-confidence; below
~−5 °F ambient it reads as "open" and holds. Both accepted — COT only matters on
boost, where the sensor is well inside its high-slope band.

Added to `platformio.ini` `[env:native]` `build_src_filter`.

### 2. `adcSensors.cpp` — ADS2 channel 3 becomes COT

- Replace the channel-3 lift-pump-pressure decode with a `cotThermistorReadC()` call
  writing `appData.compressorOutputTempC` (hold last good on rail-reject).
- **Raw passthrough**: channel 3 bypasses the `EMA_ALPHA = 0.01` smoothing that the
  other channels use — feeding a fast probe through a ~100-sample (~0.5 s) EMA would
  throw away the response time this change exists to buy. Good samples pass raw;
  rail-reject is validity gating, not filtering, and stays.
- Expose a fresh-sample signal (e.g. `AdcSensors::cotSampleReady()` latched when
  channel 3 produces a new reading, cleared by the caller) so the main loop can drive
  `cotSettle` off the real ADS cadence.

### 3. `cotSettle` fed at the native ADS rate

- The main loop calls `cotSettleStep()` whenever a fresh COT sample is ready (the ADS
  round-robin delivers channel 3 at ~200 Hz), passing the raw COT, the boost derived
  from the pressure channels, and the measured `dt` (an `elapsedMicros` reset each
  fresh COT sample). No decimation.
- Resize the two count-based buffers, which were sized for the thermocouple's ~10 Hz
  feed, so their **time** spans survive at the ADS rate (everything else in cotSettle
  is already time-based and self-adjusts):
  - `COT_SLOPE_WINDOW: 16 → 128` (0.5 s window holds up to 256 Hz; stays within the
    `uint8_t` ring indices; ~1.5 KB stack in `windowedSlopes`).
  - `COT_SETTLE_BUFFER: 512 → 2048` (~10 s capture at 200 Hz; ~16 KB static in the one
    `CotSettleState`).
- **Verify the true per-channel rate on first flash** and adjust the two constants if
  the measured cadence differs materially from ~200 Hz.

### 4. Remove the MAX31856 thermocouple

- Delete `src/sensors/cotSensor.cpp/.h`, its include, `CotSensor::Initialize()`, and
  the `if (CotSensor::update())` block in `ovgt.cpp` (its cotSettle feed moves to §3).
- Remove the `adafruit/Adafruit MAX31856 library` dependency from `platformio.ini`.
- Drop CS pin 37 from the SPI-CS-high init loop in `ovgt::setup()` (DRDY 35 / FLT 36
  were owned by `cotSensor` and are now simply unused).

### 5. Cleanup

- Remove the orphaned `liftPumpPressureHpa` field from `AppData.h` (nothing else
  reads it).

### Unchanged consumers

The CE calc (`ovgt.cpp:93`), the `cot_c` telemetry line, and the J1939 COT broadcast
(`j1939.cpp:249`) all read `appData.compressorOutputTempC` directly — they pick up the
fast raw thermistor value with no change. The `"s"` τ-measurement telemetry emit stays
as-is, now driven by the ADS-rate cotSettle feed.

## Testing

- **New** `test/test_cot_thermistor/` (native): assert `cotThermistorReadC()` against
  datasheet points — e.g. 1408 Ω → 305 °F (151.7 °C), 6954 Ω → 200 °F, 208 Ω → 485 °F —
  within ~0.5 °C, plus rail-reject returns false at ≥4.99 V and ≤0.01 V.
- **`test/test_cot_settle`** stays valid: the state-machine logic is unchanged; the
  larger buffer constants only extend capacity. Add one case feeding at ~200 Hz to
  confirm the settled-flag window and a short τ capture behave at the new rate.
- Build both environments (`pio run -e teensy41`, `pio test -e native`).

## Accepted caveats

- Cold COT (<~150 °F) is low-confidence (near 5 V rail); irrelevant to CE-on-boost.
- Peak 500 °F is just past the 485 °F table end (Steinhart-Hart extrapolates cleanly)
  and is the sensor's characterized limit — watch for drift if COT sustains >485 °F.
- `cotSettle` thresholds remain tuned to K-type noise until re-tuned on real data.
