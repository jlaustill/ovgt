#include "ADS1220_Teensy.h"

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

// Reference voltage - set to your external reference
#define VREF_EXTERNAL    5.0f  // 5V external reference

ADS1220_Teensy* ADS1220_Teensy::instancePointer = nullptr;

// Fixed constructor with correct initialization order
ADS1220_Teensy::ADS1220_Teensy()
  : _spi(nullptr), _csPin(0), _drdyPin(0), _channelCount(0),
    _gain(gain1), _rate(dataRate20SPS), _vref(VREF_EXTERNAL),
    _dmaTransferInProgress(false) {  // Now in correct order
  instancePointer = this;
  
  // Initialize channel buffers to zero
  for (int i = 0; i < MAX_CHANNELS; i++) {
    for (int j = 0; j < MAX_BUFFER_SIZE; j++) {
      _data[i].buffer[j] = 0.0f;
    }
  }
}

void ADS1220_Teensy::setVref(float vref) {
  _vref = vref;
  Serial.print("Setting reference voltage to ");
  Serial.print(_vref, 3);
  Serial.println("V");
}

void ADS1220_Teensy::begin(SPIClass& spi, uint8_t csPin, uint8_t drdyPin) {
  _spi = &spi;
  _csPin = csPin;
  _drdyPin = drdyPin;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  pinMode(_drdyPin, INPUT);
  
  _spi->begin();
  
  // Try different SPI modes to find the one that works best
  const int NUM_MODES = 4;
  SPISettings modeSettings[] = {
    SPISettings(500000, MSBFIRST, SPI_MODE0),
    SPISettings(500000, MSBFIRST, SPI_MODE1),
    SPISettings(500000, MSBFIRST, SPI_MODE2),
    SPISettings(500000, MSBFIRST, SPI_MODE3)
  };
  
  bool communicationEstablished = false;
  int workingMode = -1;
  
  for (int mode = 0; mode < NUM_MODES && !communicationEstablished; mode++) {
    Serial.print("\nTrying SPI_MODE");
    Serial.println(mode);
    
    _spiSettings = modeSettings[mode];
    
    // Reset the device
    select();
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(CMD_RESET);
    _spi->endTransaction();
    deselect();
    delay(10);
    
    // Write to REG0 (0x08 = 20 SPS)
    select();
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(CMD_WREG | (REG_CONFIG0 << 2));
    _spi->transfer(0x00); // Write 1 register
    _spi->transfer(0x08); // 20 SPS
    _spi->endTransaction();
    deselect();
    delay(5);
    
    // Read back REG0
    select();
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(CMD_RREG | (REG_CONFIG0 << 2));
    _spi->transfer(0x00); // Read 1 register
    uint8_t regValue = _spi->transfer(0x00);
    _spi->endTransaction();
    deselect();
    
    Serial.print("REG0 read: 0x");
    Serial.println(regValue, HEX);
    
    if (regValue == 0x08) {
      Serial.println("SUCCESS! Communication established.");
      communicationEstablished = true;
      workingMode = mode;
    }
    else if (regValue != 0xFF) {
      Serial.println("Partial success - got a response, but not the expected value.");
      communicationEstablished = true;
      workingMode = mode;
    }
    
    delay(100);
  }
  
  if (workingMode >= 0) {
    Serial.print("Using SPI_MODE");
    Serial.println(workingMode);
    _spiSettings = modeSettings[workingMode];
  } else {
    Serial.println("Using default SPI_MODE1");
    _spiSettings = SPISettings(500000, MSBFIRST, SPI_MODE1);
  }
  
  // Reset again with the correct mode
  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_RESET);
  _spi->endTransaction();
  deselect();
  delay(10);
  
  // Configure ADS1220 for operation with external reference
  
  // CONFIG0: Set data rate to 20 SPS, MUX to AIN0/GND (single-ended)
  uint8_t config0 = 0x08; // 20 SPS, AIN0 to GND (default MUX setting 0000)
  writeRegister(REG_CONFIG0, config0);
  
  // CONFIG1: Set gain to 1, disable temp sensor
  uint8_t config1 = 0x00; // Gain 1, disable temp sensor, no low-side power switch
  writeRegister(REG_CONFIG1, config1);
  
  // CONFIG2: Use external reference on REFP0/REFN0, disable IDAC
  // For external reference: bit 6=0, bit 5=0
  // Disable IDAC: bits 2-0 = 000
  uint8_t config2 = 0x00; // External reference on REFP0/REFN0, IDAC off
  writeRegister(REG_CONFIG2, config2);
  
  // CONFIG3: Default settings - no IDAC routing
  uint8_t config3 = 0x00;
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
  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_START_SYNC);
  _spi->endTransaction();
  deselect();
  
  // Wait for DRDY to go low
  Serial.println("Waiting for DRDY to go low...");
  uint32_t startTime = millis();
  while (digitalRead(_drdyPin) == HIGH && (millis() - startTime < 1000)) {
    // Wait for DRDY or timeout
  }
  
  if (digitalRead(_drdyPin) == LOW) {
    Serial.println("DRDY went low, reading data...");
    
    // Read the data
    select();
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(CMD_RDATA);
    
    uint8_t data[3];
    data[0] = _spi->transfer(0);
    data[1] = _spi->transfer(0);
    data[2] = _spi->transfer(0);
    
    _spi->endTransaction();
    deselect();
    
    uint32_t raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    if (raw & 0x800000) raw |= 0xFF000000; // sign extend
    
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

void ADS1220_Teensy::start(Channel ch0) {
  _channels[0] = ch0;
  _channelCount = 1;
  _currentChannelIndex = 0;

  Channel ch = _channels[_currentChannelIndex];
  uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)

  writeRegister(REG_CONFIG0, (readRegister(REG_CONFIG0) & 0x0F) | mux);

  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_START_SYNC);
  _spi->endTransaction();
  deselect();
  
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
    select();
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(CMD_RDATA);

    _dmaTransferInProgress = true;
    
    // Determine correct DMA trigger source based on SPI instance
    uint8_t dmamux_source;
    if (_spi == &SPI) {
      dmamux_source = DMAMUX_SOURCE_LPSPI4_RX;
    } else if (_spi == &SPI1) {
      dmamux_source = DMAMUX_SOURCE_LPSPI3_RX;
    } else if (_spi == &SPI2) {
      dmamux_source = DMAMUX_SOURCE_LPSPI1_RX;
    } else {
      dmamux_source = DMAMUX_SOURCE_LPSPI4_RX; // Default
    }
    
    _dma.triggerAtHardwareEvent(dmamux_source);
    _dma.enable();

    // Send 3 dummy bytes to clock out the 24-bit ADC result
    _spi->transfer(0); // Dummy bytes to trigger DMA RX
    _spi->transfer(0);
    _spi->transfer(0);
    
    // If DMA doesn't trigger for some reason, add a manual timeout
    uint32_t startTime = millis();
    while (_dmaTransferInProgress && (millis() - startTime < 100)) {
      // Wait for DMA or timeout
    }
    
    // If we timed out, handle it manually
    if (_dmaTransferInProgress) {
      Serial.println("DMA timeout - handling manually");
      _dmaTransferInProgress = false;
      _spi->endTransaction();
      deselect();
      
      // Read manually instead
      float manualReading = readDataManually();
      updateChannelAverage(_currentChannelIndex, manualReading);
      
      // Move to next channel
      _currentChannelIndex = (_currentChannelIndex + 1) % _channelCount;
      Channel ch = _channels[_currentChannelIndex];
      uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)
      
      writeRegister(REG_CONFIG0, (readRegister(REG_CONFIG0) & 0x0F) | mux);
      
      select();
      _spi->beginTransaction(_spiSettings);
      _spi->transfer(CMD_START_SYNC);
      _spi->endTransaction();
      deselect();
    }
  } else {
    Serial.println("DRDY interrupt while DMA in progress - ignoring");
  }
}

void ADS1220_Teensy::handleDataReady() {
  Serial.println("Data ready");
  _spi->endTransaction();
  deselect();

  uint32_t raw = ((_dmaRxBuffer[0] << 16) | (_dmaRxBuffer[1] << 8) | _dmaRxBuffer[2]);
  if (raw & 0x800000) raw |= 0xFF000000; // sign extend
  
  // Debug raw data
  Serial.print("Raw data bytes: 0x");
  Serial.print(_dmaRxBuffer[0], HEX);
  Serial.print(" 0x");
  Serial.print(_dmaRxBuffer[1], HEX);
  Serial.print(" 0x");
  Serial.println(_dmaRxBuffer[2], HEX);
  Serial.print("Raw value: ");
  Serial.print((int32_t)raw);
  Serial.print(" (0x");
  Serial.print(raw, HEX);
  Serial.println(")");
  
  // Use the external reference voltage instead of fixed 2.048V
  float voltage = (float)((int32_t)raw) * _vref / 0x7FFFFF;
  Serial.print("Calculated voltage: ");
  Serial.println(voltage, 6);

  updateChannelAverage(_currentChannelIndex, voltage);

  _currentChannelIndex = (_currentChannelIndex + 1) % _channelCount;
  Channel ch = _channels[_currentChannelIndex];
  uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)

  writeRegister(REG_CONFIG0, (readRegister(REG_CONFIG0) & 0x0F) | mux);

  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_START_SYNC);
  _spi->endTransaction();
  deselect();
}

// Update readDataManually as well:
float ADS1220_Teensy::readDataManually() {
  Serial.println("Reading data manually (no DMA)");
  
  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_RDATA);
  
  // Read 3 bytes directly
  uint8_t data[3];
  data[0] = _spi->transfer(0);
  data[1] = _spi->transfer(0);
  data[2] = _spi->transfer(0);
  
  _spi->endTransaction();
  deselect();
  
  // Debug raw data
  Serial.print("Raw data bytes: 0x");
  Serial.print(data[0], HEX);
  Serial.print(" 0x");
  Serial.print(data[1], HEX);
  Serial.print(" 0x");
  Serial.println(data[2], HEX);
  
  uint32_t raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  if (raw & 0x800000) raw |= 0xFF000000; // sign extend
  
  Serial.print("Raw value: ");
  Serial.print((int32_t)raw);
  Serial.print(" (0x");
  Serial.print(raw, HEX);
  Serial.println(")");
  
  // Use external reference for calculation
  float voltage = (float)((int32_t)raw) * _vref / 0x7FFFFF;
  Serial.print("Calculated voltage: ");
  Serial.println(voltage, 6);
  
  return voltage;
}

void ADS1220_Teensy::select() {
  digitalWrite(_csPin, LOW);
}

void ADS1220_Teensy::deselect() {
  digitalWrite(_csPin, HIGH);
}

void ADS1220_Teensy::writeRegister(uint8_t reg, uint8_t value) {
  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_WREG | ((reg & 0x03) << 2));
  _spi->transfer(0x00); // write one register
  _spi->transfer(value);
  _spi->endTransaction();
  deselect();
}

uint8_t ADS1220_Teensy::readRegister(uint8_t reg) {
  select();
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(CMD_RREG | ((reg & 0x03) << 2));
  _spi->transfer(0x00); // read one register
  uint8_t value = _spi->transfer(0x00);
  _spi->endTransaction();
  deselect();
  return value;
}

void ADS1220_Teensy::updateChannelAverage(uint8_t internalIndex, float newValue) {
  ChannelData& cd = _data[internalIndex];
  cd.sum -= cd.buffer[cd.index];
  cd.buffer[cd.index] = newValue;
  cd.sum += newValue;
  cd.index = (cd.index + 1) % cd.size;
  cd.average = cd.sum / cd.size;
  
  Serial.print("Channel ");
  Serial.print(_channels[internalIndex]);
  Serial.print(" new value: ");
  Serial.print(newValue, 6);
  Serial.print(", average: ");
  Serial.println(cd.average, 6);
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
    instancePointer->handleDataReady();
  }
}