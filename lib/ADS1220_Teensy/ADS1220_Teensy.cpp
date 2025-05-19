#include "ADS1220_Teensy.h"

ADS1220_Teensy* ADS1220_Teensy::instancePointer = nullptr;

// Fixed constructor with correct initialization order
ADS1220_Teensy::ADS1220_Teensy()
  : _spi(nullptr), _csPin(0), _drdyPin(0), _channelCount(0),
    _gain(gain1), _rate(dataRate20SPS), _vref(5.0f),
    _dmaTransferInProgress(false) {  // Now in correct order
  instancePointer = this;
  
  // Initialize channel buffers to zero
  for (int i = 0; i < MAX_CHANNELS; i++) {
    for (int j = 0; j < MAX_BUFFER_SIZE; j++) {
      _data[i].buffer[j] = 0.0f;
    }
  }
  
  // Initialize SPI settings
  _spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE1);
}

void ADS1220_Teensy::setVref(float vref) {
  _vref = vref;
  Serial.print("Setting reference voltage to ");
  Serial.print(_vref, 3);
  Serial.println("V");

    config2 &= ~REG_CONFIG2_VREF_MASK;
    config2 |= static_cast<uint8_t>(vref * 10);
    writeRegister(REG_CONFIG2,config2);
}

void ADS1220_Teensy::begin(SPIClass& spi, uint8_t csPin, uint8_t drdyPin) {
  _spi = &spi;
  _csPin = csPin;
  _drdyPin = drdyPin;

  pinMode(_csPin, OUTPUT);
  deselect();
  pinMode(_drdyPin, INPUT);
  
  _spi->begin();
  
  // Reset the device - based on Protocentral approach
  SPI_Command(CMD_RESET);
  delay(10); // Give some time to reset
  
  // Wait for DRDY to go low after initialization
  Serial.println("Waiting for DRDY to go low after reset...");
  uint32_t startTime = millis();
  while(digitalRead(_drdyPin) == HIGH) {
    if(millis() - startTime > 1000) {
      Serial.println("Timeout waiting for DRDY after reset!");
      break;
    }
    delay(1);
  }
  
  // Configure ADS1220 for operation with external reference
  // Using more standard approach based on Protocentral
  
  // CONFIG0: 
  // - Bits 7-4: MUX setting for AIN0 to AINN (GND)
  // - Bits 3-1: Gain setting (001 = 2)
  // - Bit 0: PGA enabled (0)
  config0 = 0x01; // AIN0 to AINN=GND, Gain 1, PGA not enabled
  
  // CONFIG1:
  // - Bits 7-5: Data rate (000 = 20 SPS)
  // - Bits 4-3: Operating mode (00 = Normal)
  // - Bit 2: Conversion mode (1 = continuous)
  // - Bit 1: Temperature sensor mode (0 = disabled)
  // - Bit 0: Current source (0 = off)
  config1 = 0x04; // 20 SPS, Normal mode, Continuous conversion, Temp sensor off, Current source off
  
  // CONFIG2:
  // - Bits 7-6: External reference (11 = Analog Supply (AVDD - AVSS_))
  // - Bits 5-4: 50/60Hz rejection (00 = no rejection)
  // - Bit 3: Low-side power switch (0 = always open)
  // - Bits 2-0: IDAC current setting (000 = off)
  config2 = 0b11000000; // External reference, 50/60Hz rejection, switch open, IDAC off
  
  // CONFIG3:
  // - Bits 7-5: IDAC1 routing (000 = disabled)
  // - Bits 4-2: IDAC2 routing (000 = disabled)
  // - Bit 1: DRDY mode (0 = DRDY pin reflects data availability)
  // - Bit 0: Reserved (0)
  config3 = 0x00; // IDAC1 disabled, IDAC2 disabled, DRDY pin indicates data ready
  
  // Write configuration registers
  writeRegister(REG_CONFIG0, config0);
  writeRegister(REG_CONFIG1, config1);
  writeRegister(REG_CONFIG2, config2);
  writeRegister(REG_CONFIG3, config3);
  
  // Print current register values for debugging
  Serial.println("ADS1220 Register Values after configuration for external reference:");
  for (int i = 0; i < 4; i++) {
    Serial.print("REG");
    Serial.print(i);
    Serial.print(": 0x");
    Serial.println(readRegister(i), HEX);
  }
  
  // Now attach the interrupt
  attachInterrupt(digitalPinToInterrupt(_drdyPin), _isrHandler, FALLING);
  Serial.printf("Attaching interrupt to pin %d (interrupt #%d)\n", _drdyPin, digitalPinToInterrupt(_drdyPin));
  
  // Do a manual test reading to verify operation
  Serial.println("Performing a manual test reading...");
  
  // Start conversion
  SPI_Command(CMD_START_SYNC);
  
  // Wait for DRDY to go low
  Serial.println("Waiting for DRDY to go low...");
  startTime = millis();
  while (digitalRead(_drdyPin) == HIGH && (millis() - startTime < 1000)) {
    // Wait for DRDY or timeout
  }
  
  if (digitalRead(_drdyPin) == LOW) {
    Serial.println("DRDY went low, reading data...");
    
    // Read the data using simplified approach
    uint8_t data[3];
    SPI.beginTransaction(_spiSettings);
    select();
    delayMicroseconds(1);
    
    SPI.transfer(CMD_RDATA);
    data[0] = SPI.transfer(0);
    data[1] = SPI.transfer(0);
    data[2] = SPI.transfer(0);
    
    delayMicroseconds(1);
    deselect();
    SPI.endTransaction();
    
    uint32_t raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    if (data[0] & 0x80) { // Check MSB for sign
        raw |= 0xFF000000; // Sign extend for negative numbers
    }
    
    // Use external reference voltage for calculation
    float voltage = (float)((int32_t)raw) * _vref / 0x7FFFFF;
    
    Serial.print("Test reading raw value: 0x");
    Serial.print(data[0], HEX);
    Serial.print(" 0x");
    Serial.print(data[1], HEX);
    Serial.print(" 0x");
    Serial.println(data[2], HEX);
    
    Serial.print("Calculated voltage: ");
    Serial.print(voltage, 6);
    Serial.println("V");
    
    // Expected value calculation for AIN0 = 3.3V with 5V reference
    float expectedRatio = 3.3f / 5.0f;
    float expectedRaw = expectedRatio * 0x7FFFFF;
    
    Serial.print("Expected ratio for 3.3V/5.0V: ");
    Serial.println(expectedRatio, 4);
    
    Serial.print("Expected raw value: ~");
    Serial.println((uint32_t)expectedRaw, HEX);
    
    // Calculate percentage error
    float percentFullScale = (float)((int32_t)raw) / 0x7FFFFF * 100.0f;
    Serial.print("Reading is ");
    Serial.print(percentFullScale, 2);
    Serial.println("% of full scale");
  } else {
    Serial.println("DRDY timeout waiting for reading!");
  }
  
  // Setup DMA
  _dma.destinationBuffer(_dmaRxBuffer, sizeof(_dmaRxBuffer));
  _dma.disableOnCompletion();
  _dma.attachInterrupt(_dmaISR);
  
  // Determine and set the correct DMA source
  if (_spi == &SPI) {
    _dma.source((volatile uint8_t&)IMXRT_LPSPI4_S.RDR);
    Serial.println("Using SPI (LPSPI4)");
  } else if (_spi == &SPI1) {
    _dma.source((volatile uint8_t&)IMXRT_LPSPI3_S.RDR);
    Serial.println("Using SPI1 (LPSPI3)");
  } else if (_spi == &SPI2) {
    _dma.source((volatile uint8_t&)IMXRT_LPSPI1_S.RDR);
    Serial.println("Using SPI2 (LPSPI1)");
  } else {
    Serial.println("WARNING: Unknown SPI instance, DMA might not work correctly");
    _dma.source((volatile uint8_t&)IMXRT_LPSPI4_S.RDR); // Default to SPI
  }
  
  Serial.println("ADS1220 initialization complete");
}


void ADS1220_Teensy::SPI_Command(uint8_t cmd) {
  SPI.beginTransaction(_spiSettings);
  select();
  delayMicroseconds(1);
  SPI.transfer(cmd);
  delayMicroseconds(1);
  deselect();
  SPI.endTransaction();
}

void ADS1220_Teensy::start(Channel ch0) {
  _channels[0] = ch0;
  _channelCount = 1;
  _currentChannelIndex = 0;

  Channel ch = _channels[_currentChannelIndex];
  uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)

  // Set mux configuration in CONFIG0 register
  // Clear bits 7-4 (MUX config) and set new MUX value
  uint8_t config0 = readRegister(REG_CONFIG0);
  config0 &= 0x0F; // Clear MUX bits (bits 7-4)
  config0 |= mux;  // Set new MUX value
  writeRegister(REG_CONFIG0, config0);

  // Start continuous conversions
  SPI_Command(CMD_START_SYNC);
  
  Serial.print("Starting ADS1220 with channel ");
  Serial.println(ch0);
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1) {
  start(ch0);
  _channels[1] = ch1;
  _channelCount = 2;
  Serial.print("Added channel ");
  Serial.println(ch1);
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1, Channel ch2) {
  start(ch0, ch1);
  _channels[2] = ch2;
  _channelCount = 3;
  Serial.print("Added channel ");
  Serial.println(ch2);
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1, Channel ch2, Channel ch3) {
  start(ch0, ch1, ch2);
  _channels[3] = ch3;
  _channelCount = 4;
  Serial.print("Added channel ");
  Serial.println(ch3);
}

void ADS1220_Teensy::setBuffer(Channel ch, uint8_t size) {
  for (uint8_t i = 0; i < _channelCount; ++i) {
    if (_channels[i] == ch && size <= MAX_BUFFER_SIZE) {
      _data[i].size = size;
      _data[i].index = 0;
      _data[i].sum = 0;
      _data[i].average = 0;
      
      // Reset the buffer to zeros
      for (int j = 0; j < size; j++) {
        _data[i].buffer[j] = 0.0f;
      }
    }
  }
}

void ADS1220_Teensy::setGain(Gain gain) {
  _gain = gain;
  uint8_t config1 = readRegister(REG_CONFIG1);
  config1 &= ~(0x07); // Clear gain bits
  config1 |= static_cast<uint8_t>(_gain);
  writeRegister(REG_CONFIG1, config1);
  
  Serial.print("Set gain to ");
  Serial.println(1 << static_cast<uint8_t>(_gain));
}

void ADS1220_Teensy::setDataRate(DataRate rate) {
  _rate = rate;
  uint8_t config0 = readRegister(REG_CONFIG0);
  config0 &= ~(0xE0); // Clear data rate bits
  config0 |= (static_cast<uint8_t>(_rate) << 5);
  writeRegister(REG_CONFIG0, config0);
  
  const int rates[] = {20, 45, 90, 175, 330, 600, 1000, 2000};
  Serial.print("Set data rate to ");
  Serial.print(rates[static_cast<uint8_t>(_rate)]);
  Serial.println(" SPS");
}

float ADS1220_Teensy::get(Channel ch) const {
  for (uint8_t i = 0; i < _channelCount; ++i) {
    if (_channels[i] == ch) {
      return _data[i].average;
    }
  }
  return 0.0f;
}

void ADS1220_Teensy::handleDrdyInterrupt() {
  Serial.println("DRDY Interrupt");
  if (!_dmaTransferInProgress) {
    // Using manual reading instead of DMA for reliability
    float voltage = readDataManually();
    
    // Update channel average
    updateChannelAverage(_currentChannelIndex, voltage);
    
    // Move to next channel
    _currentChannelIndex = (_currentChannelIndex + 1) % _channelCount;
    
    // Only update channel configuration if we have more than one channel
    if (_channelCount > 1) {
      Channel ch = _channels[_currentChannelIndex];
      uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)
      
      uint8_t config0 = readRegister(REG_CONFIG0);
      config0 &= 0x0F; // Clear MUX bits (bits 7-4)
      config0 |= mux;  // Set new MUX value
      writeRegister(REG_CONFIG0, config0);
      
      // Start next conversion
      SPI_Command(CMD_START_SYNC);
    }
  } else {
    Serial.println("DRDY interrupt while DMA in progress - ignoring");
  }
}

float ADS1220_Teensy::readDataManually() {
  Serial.println("Reading data manually");
  
  // Read data using protocentral approach
  uint8_t data[3];
  
  SPI.beginTransaction(_spiSettings);
  select();
  delayMicroseconds(1);
  
  SPI.transfer(CMD_RDATA);
  data[0] = SPI.transfer(0);
  data[1] = SPI.transfer(0);
  data[2] = SPI.transfer(0);
  
  delayMicroseconds(1);
  deselect();
  SPI.endTransaction();
  
  // Debug raw data
  Serial.print("Raw data bytes: 0x");
  Serial.print(data[0], HEX);
  Serial.print(" 0x");
  Serial.print(data[1], HEX);
  Serial.print(" 0x");
  Serial.println(data[2], HEX);
  
  uint32_t raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  if (data[0] & 0x80) { // Check MSB for sign
    raw |= 0xFF000000; // Sign extend for negative numbers
  }
  
//   Serial.print("Raw value: ");
//   Serial.print((int32_t)raw);
//   Serial.print(" (0x");
//   Serial.print(raw, HEX);
//   Serial.println(")");
  
  // Use external reference for calculation
  float voltage = (float)(((int32_t)raw * _vref / 1) / FULL_SCALE);
  Serial.print("Calculated voltage: ");
  Serial.println(voltage, 6);
  
  return voltage;
}

void ADS1220_Teensy::writeRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(_spiSettings);
  select();
  delayMicroseconds(1);
  
  SPI.transfer(CMD_WREG | (address << 2));
  SPI.transfer(value);
  
  delayMicroseconds(1);
  deselect();
  SPI.endTransaction();
}

uint8_t ADS1220_Teensy::readRegister(uint8_t reg) {
  uint8_t value;
  
  SPI.beginTransaction(_spiSettings);
  select();
  delayMicroseconds(1);
  
  SPI.transfer(CMD_RREG | (reg << 2));
  value = SPI.transfer(0);
  
  delayMicroseconds(1);
  deselect();
  SPI.endTransaction();
  
  return value;
}

void ADS1220_Teensy::updateChannelAverage(uint8_t internalIndex, float newValue) {
  ChannelData& cd = _data[internalIndex];
  cd.sum -= cd.buffer[cd.index];
  cd.buffer[cd.index] = newValue;
  cd.sum += newValue;
  cd.index = (cd.index + 1) % cd.size;
  cd.average = cd.sum / cd.size;
  
//   Serial.print("Channel ");
//   Serial.print(_channels[internalIndex]);
//   Serial.print(" new value: ");
//   Serial.print(newValue, 6);
//   Serial.print(", average: ");
//   Serial.println(cd.average, 6);
}

void ADS1220_Teensy::_isrHandler() {
  if (instancePointer) {
    instancePointer->handleDrdyInterrupt();
  }
}

void ADS1220_Teensy::_dmaISR() {
  Serial.println("DMA ISR Fired!");
  if (instancePointer) {
    instancePointer->_dma.clearInterrupt();
    instancePointer->_dmaTransferInProgress = false;
    
    // Process the data from DMA buffer
    uint32_t raw = ((instancePointer->_dmaRxBuffer[0] << 16) | 
                   (instancePointer->_dmaRxBuffer[1] << 8) | 
                    instancePointer->_dmaRxBuffer[2]);
                    
    if (instancePointer->_dmaRxBuffer[0] & 0x80) {
      raw |= 0xFF000000; // Sign extend for negative numbers
    }
    
    float voltage = (float)((int32_t)raw) * instancePointer->_vref / 0x7FFFFF;
    
    // Update channel average
    instancePointer->updateChannelAverage(instancePointer->_currentChannelIndex, voltage);
    
    // Move to next channel
    instancePointer->_currentChannelIndex = (instancePointer->_currentChannelIndex + 1) % instancePointer->_channelCount;
    
    // Update MUX configuration for next channel
    if (instancePointer->_channelCount > 1) {
      ADS1220_Teensy::Channel ch = instancePointer->_channels[instancePointer->_currentChannelIndex];
      uint8_t mux = ((uint8_t)ch) << 4;
      
      uint8_t config0 = instancePointer->readRegister(REG_CONFIG0);
      config0 &= 0x0F;
      config0 |= mux;
      instancePointer->writeRegister(REG_CONFIG0, config0);
    }
    
    // Start next conversion
    instancePointer->SPI_Command(CMD_START_SYNC);
  }
}

void ADS1220_Teensy::select() {
  digitalWriteFast(_csPin, LOW);
}

void ADS1220_Teensy::deselect() {
  digitalWriteFast(_csPin, HIGH);
}