// actuator_selftest.cpp — PARKED, ENGINE-OFF bench test of the VGT actuator.
//
// Purpose: sweep the vane demand 0% (closed) <-> 88% (open) a few times with NO
// exhaust backpressure and watch the actuator's own CAN feedback. If it tracks
// cleanly here but floats open under load on the road, the fault is actuator
// torque vs. backpressure (not electrical/mechanical/decode).
//
// This reuses the SAME Actuator scope as the real firmware (identical CAN3 @ 500k
// setup, identical command mapping, identical feedback decode) so the test is a
// faithful reproduction. It is built as a SEPARATE env (`actuator_test`); the real
// firmware (`teensy41`) is untouched. Reflash the truck with:
//     pio run -e teensy41 -t upload
//
// Run it: engine OFF, ignition/key ON (so the actuator is powered), then
//     pio run -e actuator_test -t upload

#include <Arduino.h>
#include "AppData.h"
#include "VaneConfig.h"
#include "actuator.hpp"   // Actuator_Initialize(), Actuator_Loop()

// The Actuator scope references this global; provide our own definition so we
// don't have to pull in the whole domain (ovgt.cpp) for a bench test.
AppData appData;

static const uint8_t CLOSED = VANE_CLOSED_PERCENT;  // 0
static const uint8_t OPEN   = VANE_OPEN_PERCENT;    // 88
static const uint8_t CYCLES = 4;
static const uint8_t TRACK_TOLERANCE = 5;           // %, pass window at each endpoint

// Did the actuator ever send feedback? (distinguishes "reports 0" from "silent").
static bool feedbackSeen = false;

static void noteFeedback() {
    if (appData.actuatorRawPosition != 0 || appData.actuatorStatus != 0) {
        feedbackSeen = true;
    }
}

// Command `target`% for `ms`, pumping Actuator_Loop() at 100 Hz like the real
// control loop, printing live feedback. Returns the reported position sampled at
// the end (after the vane has had time to settle).
static uint8_t holdAndWatch(uint8_t target, uint16_t ms, const char *label) {
    appData.actuatorDemandedPosition = target;
    Serial.printf(">> command %s: demand=%u%%\n", label, target);
    uint32_t start = millis();
    uint32_t lastPrint = 0;
    while (millis() - start < ms) {
        Actuator_Loop();          // send the command frame (main context, not ISR)
        noteFeedback();
        if (millis() - lastPrint >= 250) {
            lastPrint = millis();
            Serial.printf("   dem=%3u  pos=%3u  raw=%4u  motorLoad=%5u  status=0x%02X  temp=%u\n",
                          appData.actuatorDemandedPosition,
                          appData.actuatorReportedPosition,
                          appData.actuatorRawPosition,
                          appData.actuatorMotorLoad,
                          appData.actuatorStatus,
                          appData.actuatorTemp);
        }
        delay(10);
    }
    return appData.actuatorReportedPosition;
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { /* wait for host, but don't hang */ }

    Serial.println("\n===== VGT ACTUATOR SELF-TEST (parked, engine OFF) =====");
    Serial.println("Reuses the real Actuator scope: CAN3 @ 500k, cmd 0x4EA / fb 0x4EB.");
    Serial.printf("Sweeping demand %u%% (closed) <-> %u%% (open), %u cycles.\n", CLOSED, OPEN, CYCLES);
    Serial.println("Expect pos to TRACK dem within a few %% at BOTH ends with no backpressure.\n");

    appData.actuatorDemandedPosition = OPEN;  // safe default before init
    Actuator_Initialize();
    delay(200);
}

void loop() {
    static bool done = false;
    if (done) {
        // Test finished: hold OPEN (safe, no restriction) and keep feedback live.
        Actuator_Loop();
        noteFeedback();
        delay(50);
        return;
    }

    uint8_t worstCloseErr = 0;  // max |pos-0|   over cycles (want ~0)
    uint8_t worstOpenErr  = 0;  // max |pos-88|  over cycles (want ~0)

    for (uint8_t c = 1; c <= CYCLES; c++) {
        Serial.printf("--- cycle %u/%u ---\n", c, CYCLES);
        uint8_t posClosed = holdAndWatch(CLOSED, 3000, "CLOSED");
        uint8_t closeErr  = (posClosed > CLOSED) ? (posClosed - CLOSED) : (CLOSED - posClosed);
        if (closeErr > worstCloseErr) worstCloseErr = closeErr;

        uint8_t posOpen = holdAndWatch(OPEN, 3000, "OPEN");
        uint8_t openErr = (posOpen > OPEN) ? (posOpen - OPEN) : (OPEN - posOpen);
        if (openErr > worstOpenErr) worstOpenErr = openErr;

        Serial.printf("    cycle %u result: closed pos=%u (err %u), open pos=%u (err %u)\n\n",
                      c, posClosed, closeErr, posOpen, openErr);
    }

    Serial.println("===== SUMMARY =====");
    if (!feedbackSeen) {
        Serial.println("RESULT: ⚠ NO ACTUATOR FEEDBACK RECEIVED.");
        Serial.println("  The actuator never sent a 0x4EB frame. Likely NOT powered");
        Serial.println("  (turn ignition/key ON) or a CAN3 wiring/bus issue. Test inconclusive.");
    } else {
        Serial.printf("Worst closed-tracking error: %u%%  (demand 0%%)\n", worstCloseErr);
        Serial.printf("Worst open-tracking error  : %u%%  (demand %u%%)\n", worstOpenErr, OPEN);
        bool pass = (worstCloseErr <= TRACK_TOLERANCE) && (worstOpenErr <= TRACK_TOLERANCE);
        if (pass) {
            Serial.println("RESULT: ✅ PASS — actuator tracks both ends unloaded.");
            Serial.println("  => Vane hardware/electrical/decode are OK. The on-road float-open");
            Serial.println("     under boost is TORQUE vs. EXHAUST BACKPRESSURE, as suspected.");
        } else {
            Serial.println("RESULT: ❌ FAIL — actuator does NOT track even with no backpressure.");
            Serial.println("  => Fault is in the actuator/linkage/electrical itself, not just");
            Serial.println("     backpressure. Investigate the actuator directly.");
        }
    }
    Serial.println("Holding vane OPEN. Reflash real firmware: pio run -e teensy41 -t upload");
    Serial.println("===================");
    appData.actuatorDemandedPosition = OPEN;
    done = true;
}
