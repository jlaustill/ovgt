#include <Arduino.h>
#include <Adafruit_FRAM_SPI.h>
#include "fram.h"

static const uint8_t CS_PIN   = 10;
static const uint8_t HOLD_PIN = 8;
static const uint8_t WP_PIN   = 9;

static Adafruit_FRAM_SPI fram(CS_PIN);

bool Fram::Initialize() {
    pinMode(HOLD_PIN, OUTPUT);
    pinMode(WP_PIN, OUTPUT);
    digitalWrite(HOLD_PIN, HIGH);
    digitalWrite(WP_PIN, HIGH);

    if (!fram.begin()) {
        Serial.println("FRAM not found, check wiring!");
        return false;
    }
    Serial.println("FRAM initialized");
    return true;
}

bool Fram::test() {
    const uint32_t TEST_ADDR  = 0;
    const uint8_t  TEST_WRITE = 0xA5;

    fram.write8(TEST_ADDR, TEST_WRITE);
    uint8_t readBack = fram.read8(TEST_ADDR);

    if (readBack == TEST_WRITE) {
        Serial.println("FRAM test PASSED");
        return true;
    }
    Serial.print("FRAM test FAILED: wrote 0xA5, read 0x");
    Serial.println(readBack, HEX);
    return false;
}
