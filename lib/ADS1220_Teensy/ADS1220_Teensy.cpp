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

ADS1220_Teensy* ADS1220_Teensy::instancePointer = nullptr;

ADS1220_Teensy::ADS1220_Teensy()
  : _spi(nullptr), _csPin(0), _drdyPin(0), _channelCount(0),
    _gain(gain1), _rate(dataRate20SPS), _dmaTransferInProgress(false) {
  instancePointer = this;
  
  // Initialize channel buffers to zero
  for (int i = 0; i < MAX_CHANNELS; i++) {
    for (int j = 0; j < MAX_BUFFER_SIZE; j++) {
      _data[i].buffer[j] = 0.0f;
    }
  }
}

void ADS1220_Teensy::begin(SPIClass& spi, uint8_t csPin, uint8_t drdyPin) {
  _spi = &spi;
  _csPin = csPin;
  _drdyPin = drdyPin;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  pinMode(_drdyPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_drdyPin), _isrHandler, FALLING);
  Serial.printf("Attaching interrupt to pin %d (interrupt #%d)\n", _drdyPin, digitalPinToInterrupt(_drdyPin));


  _spi->begin();

  select();
  _spi->transfer(CMD_RESET);
  deselect();

  delay(5); // Wait for reset

  // Apply default configuration
  setGain(_gain);
  setDataRate(_rate);

  // DMA setup
_dma.source(_spi->hardware().fifo ? (volatile uint8_t&)_spi->hardware().fifo->RXFIFO : 
                                   (volatile uint8_t&)_spi->hardware().POPR);

  _dma.destinationBuffer(_dmaRxBuffer, sizeof(_dmaRxBuffer));
  _dma.disableOnCompletion();
  _dma.attachInterrupt(_dmaISR);
}

void ADS1220_Teensy::start(Channel ch0) {
  _channels[0] = ch0;
  _channelCount = 1;
  _currentChannelIndex = 0;

  Channel ch = _channels[_currentChannelIndex];
  uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)

  writeRegister(REG_CONFIG0, (readRegister(REG_CONFIG0) & 0x0F) | mux);

  select();
  _spi->transfer(CMD_START_SYNC);
  deselect();
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1) {
  start(ch0);
  _channels[1] = ch1;
  _channelCount = 2;
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1, Channel ch2) {
  start(ch0, ch1);
  _channels[2] = ch2;
  _channelCount = 3;
}

void ADS1220_Teensy::start(Channel ch0, Channel ch1, Channel ch2, Channel ch3) {
  start(ch0, ch1, ch2);
  _channels[3] = ch3;
  _channelCount = 4;
}

void ADS1220_Teensy::setBuffer(Channel ch, uint8_t size) {
  for (uint8_t i = 0; i < _channelCount; ++i) {
    if (_channels[i] == ch && size <= MAX_BUFFER_SIZE) {
      _data[i].size = size;
      _data[i].index = 0;
      _data[i].sum = 0;
      _data[i].average = 0;
    }
  }
}

void ADS1220_Teensy::setGain(Gain gain) {
  _gain = gain;
  uint8_t config1 = readRegister(REG_CONFIG1);
  config1 &= ~(0x07); // Clear gain bits
  config1 |= static_cast<uint8_t>(_gain);
  writeRegister(REG_CONFIG1, config1);
}

void ADS1220_Teensy::setDataRate(DataRate rate) {
  _rate = rate;
  uint8_t config0 = readRegister(REG_CONFIG0);
  config0 &= ~(0xE0); // Clear data rate bits
  config0 |= (static_cast<uint8_t>(_rate) << 5);
  writeRegister(REG_CONFIG0, config0);
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
    _spi->transfer(CMD_RDATA);

    _dmaTransferInProgress = true;
    uint8_t dmamux_source;
if (_spi == &SPI) {
    dmamux_source = DMAMUX_SOURCE_LPSPI4_RX;  // Default SPI on Teensy 4.x
} else if (_spi == &SPI1) {
    dmamux_source = DMAMUX_SOURCE_LPSPI3_RX;  // SPI1 on Teensy 4.x
} else if (_spi == &SPI2) {
    dmamux_source = DMAMUX_SOURCE_LPSPI1_RX;  // SPI2 on Teensy 4.x
}
_dma.triggerAtHardwareEvent(dmamux_source);
    _dma.enable();

    _spi->transfer(0); // Dummy bytes to trigger DMA RX
    _spi->transfer(0);
    _spi->transfer(0);
  }
}

void ADS1220_Teensy::handleDataReady() {
    Serial.println("Data ready");
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
    
  float voltage = (float)((int32_t)raw) * 2.048f / 0x7FFFFF;

  updateChannelAverage(_currentChannelIndex, voltage);

  _currentChannelIndex = (_currentChannelIndex + 1) % _channelCount;
  Channel ch = _channels[_currentChannelIndex];
  uint8_t mux = ((uint8_t)ch) << 4; // AINx to AINN (GND)

  writeRegister(REG_CONFIG0, (readRegister(REG_CONFIG0) & 0x0F) | mux);

  select();
  _spi->transfer(CMD_START_SYNC);
  deselect();
}

void ADS1220_Teensy::select() {
  digitalWrite(_csPin, LOW);
}

void ADS1220_Teensy::deselect() {
  digitalWrite(_csPin, HIGH);
}

void ADS1220_Teensy::writeRegister(uint8_t reg, uint8_t value) {
  select();
  _spi->transfer(CMD_WREG | ((reg & 0x03) << 2));
  _spi->transfer(0x00); // write one register
  _spi->transfer(value);
  deselect();
}

uint8_t ADS1220_Teensy::readRegister(uint8_t reg) {
  select();
  _spi->transfer(CMD_RREG | ((reg & 0x03) << 2));
  _spi->transfer(0x00); // read one register
  uint8_t value = _spi->transfer(0x00);
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

