#include "./ads1220.h"

volatile bool Ads1220Wrapper::dataReadyFlag = false;

Ads1220Wrapper::Ads1220Wrapper(uint8_t csPin, uint8_t dataReadyPin) {
    this->csPin = csPin;
    this->dataReadyPin = dataReadyPin;
}

void Ads1220Wrapper::setup() {
    pc_ads1220.begin(csPin, dataReadyPin);
    pc_ads1220.set_data_rate(DR_1000SPS);
    pc_ads1220.set_conv_mode_continuous();          //Set continuous conversion mode
    pc_ads1220.Start_Conv();  //Start continuous conversion mode
    enableInterruptPin();
}

void Ads1220Wrapper::dataReadyHandler() {
    dataReadyFlag = true;
}

void Ads1220Wrapper::enableInterruptPin() {
    attachInterrupt(digitalPinToInterrupt(dataReadyPin), []() { dataReadyHandler(); }, FALLING);
}

void Ads1220Wrapper::loop() {
    if (dataReadyFlag) {
        adc_data = pc_ads1220.Read_Data_Samples();
        Vout = (adc_data * 5.0*1000) / 5.0;
        dataReadyFlag = false;
    }
}