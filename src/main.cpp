#include <Arduino.h>

uint32_t lastBlink = 0;
uint8_t LED_PIN = PA1;

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, 0);
    lastBlink = millis();
}

void loop() {
    uint32_t currentMillis = millis();

    if ((currentMillis - lastBlink) >= 1000) {
        lastBlink = currentMillis;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        Serial.println("blinky blink " + (String) LED_PIN);
    }
}
