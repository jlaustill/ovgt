#include <Arduino.h>
#include "actuator.h"

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

uint8_t p = 21;
uint8_t maxp = 91;
uint8_t minp = 7;

AppData *Actuator::appData;
uint8_t Actuator::actuatorPin = 2; // Default to pin 2


void canSniff(const CAN_message_t &msg) {
    uint16_t position = (msg.buf[2] << 8) | msg.buf[3];
    position = map(position, 174, 925, 0, 100);
    uint16_t motorLoad = msg.buf[6] << 8 | msg.buf[7];
    uint8_t status = msg.buf[0];
    uint8_t temp = msg.buf[5];

    Actuator::appData->actuatorReportedPosition = position;
    Actuator::appData->actuatorMotorLoad = motorLoad;
    Actuator::appData->actuatorStatus = status;
    Actuator::appData->actuatorTemp = temp;
 
    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" Status: ");
    Serial.print(msg.buf[0]);
    Serial.print(" Temp: ");
    Serial.print(msg.buf[5]);
    Serial.print("C Load: ");
    Serial.println(motorLoad);
}

void Actuator::SetPosition(uint8_t position) {
  if (position > 100) {
    position = 100;

  } else if (position < 0) {
    position = 0;
  }
  int ap = map(position, 0, 100, minp, maxp);
  ap = map(ap, 0, 100, 7, 247);
  analogWrite(actuatorPin, ap);
}

void Actuator::Initialize(AppData *currentData, uint8_t pin) {
    appData = currentData;
    actuatorPin = pin; // Set the pin from the parameter
    
    Can0.begin();
    Can0.setBaudRate(500 * 1000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canSniff);
    Can0.mailboxStatus();

    analogWriteFrequency(actuatorPin, 300.0);
    pinMode(actuatorPin, OUTPUT);
    SetPosition(p);
}

void Actuator::Loop() {
 static uint32_t timeout = millis();
//  static boolean climbing = true;
  if ( millis() - timeout > 100 ) {
  SetPosition(p);

    // if (climbing) {
    //   if (p < 100) {
    //     p++;
    //   } else {
    //     climbing = false;
    //   }
    // } else {
    //   if (p > 0) {
    //     p--;
    //   } else {
    //     climbing = true;
    //   }
    // }

    Actuator::appData->actuatorDemandedPosition = p;
    
    timeout = millis();
  }
}
