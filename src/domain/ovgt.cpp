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

// Function to manually test the ADS1220 pins and connections
void testADS1220Hardware() {
  Serial.println("\n=== Testing ADS1220 Hardware Connections ===");
  
  // Test CS pin
  Serial.println("Testing CS pin control...");
  digitalWrite(ADS1220_CS_PIN, LOW);
  Serial.println("CS pin set LOW");
  delay(500);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  Serial.println("CS pin set HIGH");
  
  // Test DRDY pin
  Serial.print("DRDY pin state: ");
  Serial.println(digitalRead(ADS1220_DRDY_PIN));
  
  // Monitor DRDY for a few seconds
  Serial.println("Monitoring DRDY pin for 5 seconds...");
  unsigned long startTime = millis();
  int transitions = 0;
  bool lastState = digitalRead(ADS1220_DRDY_PIN);
  
  while (millis() - startTime < 5000) {
    bool currentState = digitalRead(ADS1220_DRDY_PIN);
    if (currentState != lastState) {
      lastState = currentState;
      transitions++;
      Serial.print("DRDY pin transition detected! State: ");
      Serial.println(currentState);
    }
    delay(1);
  }
  
  Serial.print("Total DRDY transitions in 5 seconds: ");
  Serial.println(transitions);
  
  if (transitions == 0) {
    Serial.println("WARNING: No DRDY transitions detected. This may indicate a hardware issue.");
    Serial.println("Check the DRDY pin connection and ADS1220 power.");
  }
  
  Serial.println("=== Hardware Test Complete ===\n");
}

void ovgt::debugLog() {
  Serial.print("Vout in mV : ");
  float value = ads.get(ADS1220_Teensy::AN0);
  Serial.print(value * 1000.0f);  // Convert to mV for display
  
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
  while (!Serial && millis() < 3000); // Wait for serial connection with timeout
  
  Serial.println("\n\n=== ADS1220 Test with External Reference ===");
  Serial.println("Setup: Using 5V external reference, AIN0 connected to 3.3V");
  
  // Initialize pins
  pinMode(ADS1220_CS_PIN, OUTPUT);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  pinMode(ADS1220_DRDY_PIN, INPUT);
  
  // Test hardware connections
  testADS1220Hardware();
  
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