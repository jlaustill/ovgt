#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>

// ADS1220 command definitions
#define CMD_RESET        0x06
#define CMD_START_SYNC   0x08
#define CMD_POWERDOWN    0x02
#define CMD_RDATA        0x10
#define CMD_WREG         0x40
#define CMD_RREG         0x20

// ADS1220 register addresses
#define REG_CONFIG0      0x00
#define REG_CONFIG1      0x01
#define REG_CONFIG2      0x02
#define REG_CONFIG3      0x03

#define REG_CONFIG2_VREF_MASK            0xC0

#define FULL_SCALE (((long int)1<<23)-1)

class ADS1220_Teensy {
public:
  // Gain settings (global)
  enum Gain {
    gain1 = 0,
    gain2,
    gain4,
    gain8,
    gain16,
    gain32,
    gain64,
    gain128
  };

  // Data rate settings (global)
  enum DataRate {
    dataRate20SPS = 0,
    dataRate45SPS,
    dataRate90SPS,
    dataRate175SPS,
    dataRate330SPS,
    dataRate600SPS,
    dataRate1000SPS,
    dataRate2000SPS
  };

  // Channels (AINx as single-ended inputs)
  enum Channel {
    AN0 = 0,
    AN1,
    AN2,
    AN3
  };

  ADS1220_Teensy();

  void begin(SPIClass& spi, uint8_t csPin, uint8_t drdyPin);

  // Start up to 4 channels – chaining overloads for clarity
  void start(Channel ch0);
  void start(Channel ch0, Channel ch1);
  void start(Channel ch0, Channel ch1, Channel ch2);
  void start(Channel ch0, Channel ch1, Channel ch2, Channel ch3);

  // Configure buffer depth per channel (averaging)
  void setBuffer(Channel ch, uint8_t size);

  // Set PGA gain (global)
  void setGain(Gain gain);

  // Set data rate (global)
  void setDataRate(DataRate rate);
  
  // Set reference voltage (for calculations)
  void setVref(float vref);

  // Send a single command to the ADS1220
  void SPI_Command(uint8_t cmd);

  // Get the latest averaged value (no blocking)
  float get(Channel ch) const;
  
  // Read data manually (for debugging)
  float readDataManually();

  // To be called from ISR or background task (DMA completion or DRDY)
  void handleDataReady();

  // Called by DRDY ISR trampoline
  void handleDrdyInterrupt();

private:
  uint8_t config0;
  uint8_t config1;
  uint8_t config2;
  uint8_t config3;
  static const uint8_t MAX_CHANNELS = 4;
  static const uint8_t MAX_BUFFER_SIZE = 64;

  SPISettings _spiSettings;
  SPIClass* _spi;
  uint8_t _csPin;
  uint8_t _drdyPin;
  uint8_t _channelCount;
  Gain _gain;
  DataRate _rate;
  float _vref; // Reference voltage for calculations
  volatile bool _dmaTransferInProgress;

  Channel _channels[MAX_CHANNELS];

  struct ChannelData {
    float buffer[MAX_BUFFER_SIZE];
    uint8_t index = 0;
    uint8_t size = 8; // default
    float sum = 0.0f;
    float average = 0.0f;
  } _data[MAX_CHANNELS];


  void select();
  void deselect();
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);

  void updateChannelAverage(uint8_t internalIndex, float newValue);
  float readADC();

  uint8_t _currentChannelIndex = 0;

  static ADS1220_Teensy* instancePointer;

  DMAChannel _dma;
  uint8_t _dmaRxBuffer[3];

  static void _isrHandler();
  static void _dmaISR();
};