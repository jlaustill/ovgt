#include <Arduino.h>
#include "actuator.h"

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

uint8_t p = 0;


void canSniff(const CAN_message_t &msg) {
 uint16_t position = (msg.buf[2] << 8) | msg.buf[3];
 position = map(position, 174, 925, 0, 100);
 uint16_t motorLoad = msg.buf[6] << 8 | msg.buf[7];
 
 Serial.print("Position: ");
 Serial.print(position);
 Serial.print(" Status: ");
 Serial.print(msg.buf[0]);
 Serial.print(" Temp: ");
 Serial.print(msg.buf[5]);
 Serial.print("C Load: ");
 Serial.println(motorLoad);
}

void Actuator::Initialize() {
  Can0.begin();
  Can0.setBaudRate(500 * 1000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  analogWriteFrequency(2, 300.0);
  pinMode(2, OUTPUT);
  analogWrite(2, p);
}

void Actuator::Loop() {
 static uint32_t timeout = millis();
 static boolean climbing = true;
  if ( millis() - timeout > 100 ) {
    uint8_t ap = map(p, 0, 100, 7, 247);
    analogWrite(2, ap);

    if (climbing) {
      if (p < 100) {
        p++;
      } else {
        climbing = false;
      }
    } else {
      if (p > 0) {
        p--;
      } else {
        climbing = true;
      }
    }
    
    timeout = millis();
  }
}