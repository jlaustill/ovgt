#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include <ADS1220_Teensy.h>

#include "data/ads1220.h"

#define ADS1220_CS_PIN    10
#define ADS1220_DRDY_PIN  41

ADS1220_Teensy ads = ADS1220_Teensy();

float ADS1220Temperature;
float Vout = 0.0;

volatile bool new_data = false;
uint8_t sensor = 0;

int16_t adc0 = 0;
int16_t adc1 = 0;
int16_t adc2 = 0;

unsigned long ovgt::lastMillis = 0;
unsigned long ovgt::thisMillis;
unsigned long ovgt::thisDuration;
unsigned long ovgt::count;
unsigned long ovgt::loopCountLastMillis = 0;
unsigned long fastLoopCountLastMillis = 0;
char buffer[21];  // 20 chars + null terminator
AppData ovgt::appData;


IntervalTimer debugTimer;

void ovgt::debugLog() {
    Serial.print("Vout in mV : ");
    Serial.print(ads.get(ADS1220_Teensy::AN0));

    Serial.print(" Cycles/Second: ");
    Serial.print(count);
    count = 0;

    Serial.println("");
}

void ovgt::setup() {
    Serial.begin(115200);

    ads.begin(SPI, ADS1220_CS_PIN, ADS1220_DRDY_PIN);
    ads.start(ADS1220_Teensy::AN0, ADS1220_Teensy::AN1, ADS1220_Teensy::AN2, ADS1220_Teensy::AN3);

    lastMillis = millis();
    thisMillis = millis();
    count = 0;
    thisDuration = 0;
    
    Actuator::Initialize(&ovgt::appData);

    debugTimer.begin(debugLog, 1000000); // 1 second interval

    Serial.println("Setup complete");
}

void ovgt::loop() {
    count++;

    Actuator::Loop();
}