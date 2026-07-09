#ifndef AppData_h
#define AppData_h

#include <Arduino.h>

struct AppData {
    uint8_t actuatorTemp;
    volatile uint8_t actuatorDemandedPosition;
    uint8_t actuatorReportedPosition;
    uint16_t actuatorRawPosition;
    uint16_t actuatorMotorLoad;
    uint8_t actuatorStatus;
    uint16_t compressorOutputPressureHpaa;
    float compressorInputTempC;
    uint16_t compressorInputPressureHpaa;
    uint16_t turbineInputPressureHpa;
    float turbineInputVoltage;
    float tipZeroVoltage;
    int16_t turbineInletTempC;
    float compressorOutputTempC;
    int16_t turbineOutletTempC;
    int16_t oilTempC;
    uint16_t oilPressureHpa;
    uint16_t turbineOutPressureHpa;
    uint16_t liftPumpPressureHpa;  // J1939 SPN 94 broadcast only; no local sensor wired
    bool pgFault;
    // J1939-received engine/transmission state (decoded in j1939Sniff)
    uint8_t torqueConverterLockupStatus;  // SPN 573, 2-bit (0-3), 1 = engaged
    uint8_t acceleratorPedalPercent;      // SPN 91, 0-100
    uint8_t engineLoadPercent;            // SPN 92, 0-250
    volatile uint32_t lastEec2RxMs;       // millis() of last EEC2 (PGN 61443) receive
    volatile uint32_t lastEtc1RxMs;       // millis() of last ETC1 (PGN 61442) receive
    bool exhaustBrakeActive;              // true while the exhaust brake owns the actuator

    // --- J1939 broadcast logging (decoded in j1939Sniff; see logging spec) ---
    volatile uint16_t engineRpm;              // SPN 190
    volatile int8_t   driverDemandTorquePct;  // SPN 512
    volatile int8_t   actualTorquePct;        // SPN 513
    volatile int16_t  intakeAirTempC;         // SPN 105
    volatile uint16_t j1939BoostKpa;          // SPN 102 (cross-check)
    volatile uint16_t preTurboKpa;            // SPN 108
    volatile float    systemVoltage;          // SPN 168
    volatile int16_t  coolantTempC;           // SPN 110
    volatile int16_t  engineOilTempC;         // SPN 175
    volatile uint16_t oilPressureKpaJ1939;    // SPN 100
    volatile uint16_t outputShaftRpm;         // SPN 191
    volatile uint16_t inputShaftRpm;          // SPN 161
    volatile uint8_t  clutchSlipPct;          // SPN 522
    volatile uint8_t  selectedGear;           // SPN 524 (raw gear byte value)
    volatile uint8_t  currentGear;            // SPN 523 (raw gear byte value)
    volatile uint16_t gearRatioMilli;         // SPN 526
    // Raw FF/FE J1939 validity status per signal (J1939RawStatus as uint8_t; ISR-set).
    // Convention: <signal>Raw holds the *status*; the plain field holds the *value*.
    volatile uint8_t  engineRpmRaw, torqueRaw, intakeAirRaw, boostRaw, preTurboRaw,
                      systemVoltageRaw, coolantRaw, oilTempRaw, oilPressRaw,
                      outputShaftRaw, inputShaftRaw, clutchSlipRaw,
                      selectedGearRaw, currentGearRaw, gearRatioRaw;
    // Per-PGN last-receipt millis() for freshness/online gating
    volatile uint32_t lastEec1RxMs, lastIc1RxMs, lastAmbRxMs, lastVep1RxMs,
                      lastEt1RxMs, lastEflRxMs, lastEtc2RxMs;
    // Domain-online latch timestamps (set once on first native gating frame; 0 = not yet)
    volatile uint32_t engineOnlineAtMs, transOnlineAtMs;
};

extern AppData appData;

static const bool ACTUATOR_MODE_CAN = true;

#endif