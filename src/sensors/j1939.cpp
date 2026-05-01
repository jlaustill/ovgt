#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "j1939.h"
#include "AppData.h"

// CAN2 = Teensy 4.1 pins 0 (RX) / 1 (TX). J1939 @ 250 kbps, 29-bit extended IDs.
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1939;

static const uint8_t  SA              = 0x01;
static const uint32_t PGN_EGT         = 0xFEF6; // SPN 173 — Exhaust Gas Temperature
static const uint32_t PGN_OIL_TEMP    = 0xFEEE; // SPN 175 (bytes 2-3) + SPN 176 (bytes 4-5) — Engine Oil Temperature
static const uint32_t PGN_FLUID_PRESS = 0xFEEF; // SPN 94 (byte 0) fuel, SPN 100 (byte 3) oil pressure

static uint32_t lastTx = 0;

static void j1939Sniff(const CAN_message_t &msg) {
    // TODO: decode incoming PGNs into appData fields
    // J1939 29-bit ID: priority[28:26] | DP[25:24] | PF[23:16] | PS[15:8] | SA[7:0]
    // PGN = (DP << 16) | (PF << 8) | (PF >= 240 ? PS : 0)
    (void)msg;
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

static void transmit() {
    uint8_t buf[8];

    // EGT — SPN 173, PGN 65270, bytes 5-6, encoding: (temp_c + 273) * 32
    memset(buf, 0xFF, 8);
    uint16_t egtRaw = (uint16_t)((appData.turbineInletTempC + 273) * 32);
    buf[4] = egtRaw & 0xFF;
    buf[5] = egtRaw >> 8;
    sendPgn(PGN_EGT, buf);

    // Oil Temp — PGN 65262, encoding: (temp_c + 273) * 32
    // SPN 175 (Engine Oil Temp 1): bytes 2-3
    // SPN 176 (Turbocharger Oil Temp): bytes 4-5
    memset(buf, 0xFF, 8);
    uint16_t oilTmpRaw = (uint16_t)((appData.oilTempC + 273) * 32);
    buf[2] = oilTmpRaw & 0xFF;
    buf[3] = oilTmpRaw >> 8;
    buf[4] = oilTmpRaw & 0xFF;
    buf[5] = oilTmpRaw >> 8;
    sendPgn(PGN_OIL_TEMP, buf);

    // Fluid pressures — PGN 65263 (0xFEEF), encoding: kPa / 4
    // SPN 94 (byte 0): fuel delivery pressure
    // SPN 100 (byte 3): engine oil pressure
    memset(buf, 0xFF, 8);
    buf[0] = (uint8_t)((appData.liftPumpPressureHpa / 10) / 4);
    buf[3] = (uint8_t)((appData.oilPressureHpa / 10) / 4);
    sendPgn(PGN_FLUID_PRESS, buf);
}

void J1939::Loop() {
    Can1939.events();
    uint32_t now = millis();
    if (now - lastTx >= 100) {
        lastTx = now;
        transmit();
    }
}
