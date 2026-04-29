#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "j1939.h"
#include "AppData.h"

// CAN2 = Teensy 4.1 pins 0 (RX) / 1 (TX). J1939 @ 250 kbps, 29-bit extended IDs.
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1939;

static volatile uint32_t rxCount = 0;

static void j1939Sniff(const CAN_message_t &msg) {
    rxCount++;

    // TODO(user): decode PGN here. For now just dump raw frames.
    // J1939 29-bit ID layout:
    //   Priority    : bits 26..28  (3 bits)
    //   Reserved/DP : bits 24..25  (2 bits)
    //   PDU Format  : bits 16..23  (PF)
    //   PDU Specific: bits  8..15  (PS) -> destination addr if PF<240, else group extension
    //   Source Addr : bits  0..7
    // PGN = (DP << 16) | (PF << 8) | (PF >= 240 ? PS : 0)

    Serial.print("J1939 ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" ext=");
    Serial.print(msg.flags.extended);
    Serial.print(" DLC=");
    Serial.print(msg.len);
    Serial.print(" data=");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.buf[i] < 0x10) Serial.print('0');
        Serial.print(msg.buf[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void J1939::Initialize() {
    Can1939.begin();
    Can1939.setBaudRate(250 * 1000);
    Can1939.setMaxMB(16);
    Can1939.enableFIFO();
    Can1939.enableFIFOInterrupt();
    Can1939.onReceive(j1939Sniff);
    Can1939.mailboxStatus();
    Serial.println("J1939 (CAN2 @ 250kbps) initialized");
}

void J1939::Loop() {
    Can1939.events();
    // FIFO interrupt handles RX; keep a heartbeat so silence is obvious.
    static uint32_t lastReport = 0;
    if (millis() - lastReport >= 2000) {
        lastReport = millis();
        noInterrupts();
        uint32_t n = rxCount;
        rxCount = 0;
        interrupts();
        Serial.print("J1939 frames in last 2s: ");
        Serial.println(n);
    }
}
