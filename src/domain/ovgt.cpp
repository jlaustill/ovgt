#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include <LiquidCrystal_I2C.h>
#include "AppData.h"

#include <SPI.h>


int32_t adc_data;
float ADS1220Temperature;
volatile bool drdyIntrFlag = false;
float Vout = 0.0;

volatile bool new_data = false;
uint8_t sensor = 0;


LiquidCrystal_I2C lcd(0x27, 20, 4);

unsigned long ovgt::lastMillis = 0;
unsigned long ovgt::thisMillis;
unsigned long ovgt::thisDuration;
unsigned long ovgt::count;
unsigned long ovgt::loopCountLastMillis = 0;
unsigned long fastLoopCountLastMillis = 0;
char buffer[21];  // 20 chars + null terminator
AppData ovgt::appData;


void ovgt::setup() {
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();

    lastMillis = millis();
    thisMillis = millis();
    count = 0;
    thisDuration = 0;
    
    Actuator::Initialize(&ovgt::appData);

    Serial.println("Setup complete");
}

void ovgt::loop() {
    thisMillis = millis();
    thisDuration = thisMillis - lastMillis;
    count++;
    lastMillis = thisMillis;

    Actuator::Loop();
    

    if (thisMillis - loopCountLastMillis > 1000) {
        Serial.print("Vout in mV : ");
        Serial.print(Vout);
        Serial.print("  32bit HEX : ");
        Serial.print(adc_data,HEX);
        Serial.print("  Temp in °C : ");
        Serial.println(ADS1220Temperature,5);  

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