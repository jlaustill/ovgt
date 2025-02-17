#include <Arduino.h>
// #include <Adafruit_MAX31855.h>
#include "ovgt.h"
#include "display/actuator.h"
#include <LiquidCrystal_I2C.h>
#include "AppData.h"
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 41;

volatile bool new_data = false;
uint8_t sensor = 0;

void NewDataReadyISR() {
  new_data = true;
}

int16_t adc0 = 0;
int16_t adc1 = 0;
int16_t adc2 = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

unsigned long ovgt::lastMillis = 0;
unsigned long ovgt::thisMillis;
unsigned long ovgt::thisDuration;
unsigned long ovgt::count;
unsigned long ovgt::loopCountLastMillis = 0;
unsigned long fastLoopCountLastMillis = 0;
char buffer[21];  // 20 chars + null terminator
AppData ovgt::appData;

// Adafruit_MAX31855 thermocoupleInputTemp(9);

void ovgt::setup() {
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();

    lastMillis = millis();
    thisMillis = millis();
    count = 0;
    thisDuration = 0;


//   if (!thermocoupleInputTemp.begin()) {
//     Serial.println("ERROR INITING thermocoupleInputTemp");
//   } else {
//     Serial.println("thermocoupleInputTemp initialized");
//   }
    
    Actuator::Initialize(&ovgt::appData);



    if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    }

    pinMode(READY_PIN, INPUT);
    // We get a falling edge every time a new sample is ready.
    attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

    Serial.println("Setup complete");
}

void ovgt::loop() {
    thisMillis = millis();
    thisDuration = thisMillis - lastMillis;
    count++;
    lastMillis = thisMillis;

    Actuator::Loop();

    if (thisMillis - fastLoopCountLastMillis > 1) {
        if (new_data) {
        // Read the latest data.
            if (sensor == 0) {
                adc0 = ads.getLastConversionResults();
            } else if (sensor == 1) {
                adc1 = ads.getLastConversionResults();
            } else if (sensor == 2) {
                adc2 = ads.getLastConversionResults();
            }
            sensor++;
            if (sensor > 2) {
                sensor = 0;
            }
        // Clear the flag.
        new_data = false;
        //   Serial.print("ADC0: "); Serial.print(adc0); Serial.print(", ADC1: "); Serial.print(adc1); Serial.print(", ADC2: "); Serial.println(adc2);
        } else {
            if (sensor == 0) {
                // Start continuous conversions.
                ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/true);
            } else if (sensor == 1) {
                ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/true);
            } else if (sensor == 2) {
                ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, /*continuous=*/true);
            }
        }

        fastLoopCountLastMillis = thisMillis;
        // Serial.println("Fast loop");
    }
    

    if (thisMillis - loopCountLastMillis > 1000) {
          // basic readout test, just print the current temp
        // Serial.print("Internal Temp = ");
        // Serial.println(thermocoupleInputTemp.readInternal());

        // double c = thermocoupleInputTemp.readCelsius();
        // if (isnan(c)) {
        //     Serial.println("Thermocouple fault(s) detected!");
        //     uint8_t e = thermocoupleInputTemp.readError();
        //     if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
        //     if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
        //     if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
        // } else {
        //     Serial.print("C = ");
        //     Serial.println(c + 35);
        // }


        lcd.setCursor(0, 0);
        snprintf(buffer, 21, "%2d C Motor Load %3d", ovgt::appData.actuatorTemp, ovgt::appData.actuatorMotorLoad);
        lcd.print(buffer);

        lcd.setCursor(0, 1);
        snprintf(buffer, 21, "Status %1d L/S %d", ovgt::appData.actuatorStatus, count);
        lcd.print(buffer);
        
        lcd.setCursor(0, 2);
        snprintf(buffer, 21, "Demanded %d   ", (100 - ovgt::appData.actuatorDemandedPosition));
        lcd.print(buffer);

        lcd.setCursor(0, 3);
        snprintf(buffer, 21, "Reported %d   ", ovgt::appData.actuatorReportedPosition);
        lcd.print(buffer);


        Serial.print("Loop count/Sec: ");
        Serial.print(count);
        Serial.println("");
        count = 0;
        loopCountLastMillis = thisMillis;
    }
}