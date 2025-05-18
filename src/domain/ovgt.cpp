#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include <ADS1220_Teensy.h>
#include "data/ads1220.h"

#define ADS1220_CS_PIN    9   // Your CS pin is on 9 based on previous debugging
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
  
  // Test with a manual read
  Serial.print(" | Manual Read: ");
  float manualRead = ads.readDataManually();
  Serial.print(manualRead, 6);
  
  Serial.println("");
}

void ovgt::setup() {
  Serial.begin(115200);
  delay(500); // Give some time for serial connection
  
  Serial.println("\n\n=== ADS1220 Test with External Reference ===");
  Serial.println("Setup: Using 5V external reference, AIN0 connected to 3.3V");
  
  // Initialize pins
  pinMode(ADS1220_CS_PIN, OUTPUT);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  pinMode(ADS1220_DRDY_PIN, INPUT);
  
  // Start SPI
  SPI.begin();
  delay(100);
  
  // Set the reference voltage to 5.0V
  ads.setVref(5.0);
  
  // Initialize ADS1220
  ads.begin(SPI, ADS1220_CS_PIN, ADS1220_DRDY_PIN);
  
  // Configure channels - only use channel 0 for now
  ads.start(ADS1220_Teensy::AN0);
  
  lastMillis = millis();
  thisMillis = millis();
  count = 0;
  thisDuration = 0;
      
  Actuator::Initialize(&ovgt::appData);
  
  debugTimer.begin(debugLog, 1000000); // 1 second interval
  
  Serial.println("Setup complete - expecting to see readings around 3.3V (66% of 5V reference)");
}

void ovgt::loop() {
  count++;
  
  Actuator::Loop();
}