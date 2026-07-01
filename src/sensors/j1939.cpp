#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "j1939.h"
#include "AppData.h"
#include "j1939Decode.h"
#include "j1939Encode.h"

// CAN2 = Teensy 4.1 pins 0 (RX) / 1 (TX). J1939 @ 250 kbps, 29-bit extended IDs.
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1939;

static const uint8_t  SA              = 0x01;
static const uint32_t PGN_AMB         = 65270; // AMB — SPN 102 (byte 2, boost), SPN 173 (bytes 6-7, EGT)
static const uint32_t PGN_OIL_TEMP    = 65262; // SPN 175 (bytes 3-4) + SPN 176 (bytes 5-6) — Engine Oil Temperature
static const uint32_t PGN_FLUID_PRESS = 65263; // SPN 94 (byte 1) fuel, SPN 100 (byte 4) oil pressure
static const uint32_t PGN_EEC2 = 61443; // EEC2 — SPN 91 throttle, SPN 92 load
static const uint32_t PGN_ETC1 = 61442; // ETC1 — SPN 573 torque converter lockup

// Standard turbo/intake PGNs (each carries our signal in bytes 1-2, turbo 2-4
// siblings left 0xFF = not available). See docs/j1939-spn-mapping.md.
static const uint32_t PGN_ENGINE_INFO     = 65170; // SPN 1209 exhaust gas (turbine inlet) pressure, bytes 2-3
static const uint32_t PGN_INTAKE_MANIFOLD = 65190; // SPN 1127 turbo 1 boost pressure, bytes 1-2
static const uint32_t PGN_TURBO_INFO_2    = 65178; // SPN 1172 compressor intake temp, bytes 1-2
static const uint32_t PGN_TURBO_INFO_3    = 65177; // SPN 1176 compressor intake pressure, bytes 1-2
static const uint32_t PGN_TURBO_INFO_4    = 65176; // SPN 1180 turbine intake temp, bytes 1-2
static const uint32_t PGN_TURBO_INFO_6    = 64979; // SPN 2629 compressor outlet temp, bytes 1-2

static uint32_t lastTx100ms = 0;
static uint32_t lastTx500ms = 0;
static uint32_t lastTx1000ms = 0;

// Write a 2-byte little-endian J1939 field at buf[pos] (0-based).
static void put16(uint8_t *buf, uint8_t pos, uint16_t value) {
    buf[pos] = value & 0xFF;
    buf[pos + 1] = value >> 8;
}

static void j1939Sniff(const CAN_message_t &msg) {
    uint32_t pgn = pgnFromCanId(msg.id);
    if (pgn == PGN_EEC2) {
        appData.acceleratorPedalPercent = decodeAcceleratorPedalPercent(msg.buf);
        appData.engineLoadPercent = decodeEngineLoadPercent(msg.buf);
        appData.lastEec2RxMs = millis();
    } else if (pgn == PGN_ETC1) {
        appData.torqueConverterLockupStatus = decodeTorqueConverterLockup(msg.buf);
        appData.lastEtc1RxMs = millis();
    }
}

void J1939::Initialize() {
    Can1939.begin();
    Can1939.setBaudRate(250 * 1000);
    Can1939.setMaxMB(16);
    // Non-FIFO default: MB0-7 RX_EMPTY, MB8-15 TX_INACTIVE
    Can1939.enableMBInterrupts();   // RX full + TX done interrupts on all MBs
    Can1939.onReceive(j1939Sniff);
    Serial.println("J1939 (CAN2 @ 250kbps) initialized");
}

static void sendPgn(uint32_t pgn, const uint8_t* data) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id = (6UL << 26) | (pgn << 8) | SA;
    msg.len = 8;
    memcpy(msg.buf, data, 8);
    Can1939.write(msg);
}

static void transmit100ms() {
    uint8_t buf[8];

    // AMB — PGN 65270
    // SPN 102 (byte 2): intake manifold pressure, 2 kPa/bit
    // SPN 173 (bytes 6-7): EGT, 0.03125 deg C/bit, offset -273
    memset(buf, 0xFF, 8);
    buf[1] = (uint8_t)((appData.compressorOutputPressureHpaa - appData.compressorInputPressureHpaa) / 10 / 2);
    uint16_t egtRaw = (uint16_t)((appData.turbineInletTempC + 273) * 32);
    buf[5] = egtRaw & 0xFF;
    buf[6] = egtRaw >> 8;
    sendPgn(PGN_AMB, buf);

    // Oil Temp — PGN 65262
    // SPN 175 (bytes 3-4): Engine Oil Temp 1, 0.03125 deg C/bit, offset -273
    // SPN 176 (bytes 5-6): Turbocharger Oil Temp, same encoding
    memset(buf, 0xFF, 8);
    uint16_t oilTmpRaw = (uint16_t)((appData.oilTempC + 273) * 32);
    buf[2] = oilTmpRaw & 0xFF;
    buf[3] = oilTmpRaw >> 8;
    buf[4] = oilTmpRaw & 0xFF;
    buf[5] = oilTmpRaw >> 8;
    sendPgn(PGN_OIL_TEMP, buf);

    // Engine Information — PGN 65170 (standard rate 100 ms)
    // SPN 1209 (bytes 2-3): exhaust gas / turbine inlet (drive) pressure
    memset(buf, 0xFF, 8);
    put16(buf, 1, j1939EncodeTurboPressureRaw((float)appData.turbineInputPressureHpa));
    sendPgn(PGN_ENGINE_INFO, buf);
}

static void transmit500ms() {
    uint8_t buf[8];

    // Fluid pressures — PGN 65263
    // SPN 94 (byte 1): fuel delivery pressure, 4 kPa/bit
    // SPN 100 (byte 4): engine oil pressure, 4 kPa/bit
    memset(buf, 0xFF, 8);
    buf[0] = (uint8_t)(appData.liftPumpPressureHpa / 10 / 4);
    buf[3] = (uint8_t)(appData.oilPressureHpa / 10 / 4);
    sendPgn(PGN_FLUID_PRESS, buf);

    // Intake Manifold Information 1 — PGN 65190 (standard rate 500 ms)
    // SPN 1127 (bytes 1-2): turbo 1 boost pressure, 0.125 kPa/bit
    memset(buf, 0xFF, 8);
    float boostHpa = (float)appData.compressorOutputPressureHpaa
                   - (float)appData.compressorInputPressureHpaa;
    put16(buf, 0, j1939EncodeBoostRaw(boostHpa));
    sendPgn(PGN_INTAKE_MANIFOLD, buf);
}

static void transmit1000ms() {
    uint8_t buf[8];

    // Turbocharger Information 4 — PGN 65176: SPN 1180 turbine intake temp
    memset(buf, 0xFF, 8);
    put16(buf, 0, j1939EncodeTemperatureRaw((float)appData.turbineInletTempC));
    sendPgn(PGN_TURBO_INFO_4, buf);

    // Turbocharger Information 2 — PGN 65178: SPN 1172 compressor intake temp (CIT)
    memset(buf, 0xFF, 8);
    put16(buf, 0, j1939EncodeTemperatureRaw((float)appData.compressorInputTempC));
    sendPgn(PGN_TURBO_INFO_2, buf);

    // Turbocharger Information 6 — PGN 64979: SPN 2629 compressor outlet temp (COT)
    memset(buf, 0xFF, 8);
    put16(buf, 0, j1939EncodeTemperatureRaw((float)appData.compressorOutputTempC));
    sendPgn(PGN_TURBO_INFO_6, buf);

    // Turbocharger Information 3 — PGN 65177: SPN 1176 compressor intake pressure (CIP)
    memset(buf, 0xFF, 8);
    put16(buf, 0, j1939EncodeTurboPressureRaw((float)appData.compressorInputPressureHpaa));
    sendPgn(PGN_TURBO_INFO_3, buf);
}

void J1939::Loop() {
    Can1939.events();
    uint32_t now = millis();
    if (now - lastTx100ms >= 100) {
        lastTx100ms = now;
        transmit100ms();
    }
    if (now - lastTx500ms >= 500) {
        lastTx500ms = now;
        transmit500ms();
    }
    if (now - lastTx1000ms >= 1000) {
        lastTx1000ms = now;
        transmit1000ms();
    }
}
