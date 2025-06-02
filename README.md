# OVGT - Open Source VGT Controller

![PlatformIO](https://img.shields.io/badge/platformio-teensy-orange)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

An open-source Variable Geometry Turbo controller for Teensy 4.1 and MicroMod Teensy platforms.

## Hardware Requirements

### Supported Microcontrollers
- Teensy 4.1 (primary development target)
- MicroMod Teensy (secondary target)

### Required Components
- [SN65HVD232DR CAN Transceiver](https://www.ti.com/product/SN65HVD232) (3.3V compatible)

### Optional Components
- 20x4 I2C LCD Display (for debugging or status output)
- [FRAM Memory](https://www.cypress.com/products/serial-fram) (for non-volatile config/logging)

## System Overview

This project uses a MicroMod Teensy for a high-performance, real-time embedded control and data acquisition system, designed to outperform typical OEM sensor setups. The system supports pressure, temperature, RPM, and CAN-based sensors, all integrated through high-resolution ADCs and SPI thermocouple readers.

During prototyping, a Teensy 4.1 is used via its onboard micro-USB port. Final deployments use the MicroMod Teensy, utilizing the onboard USB D+/D− pads routed through an ESD protection IC.

### Sensor + I/O Breakdown

| Function                    | Interface   | IC/Component                  | Notes |
|-----------------------------|-------------|-------------------------------|-------|
| **CAN - J1939**             | CAN via UART (CRX3/CTX3) | SN65HVD232DR         | Shielded, standard CAN |
| **CAN - Actuator**          | CAN via UART (CRX1/CTX1) | SN65HVD232DR         | Isolated from J1939 bus |
| **PWM Output**              | PWM Pin     | TIP120                        | For VGT actuator control |
| **Turbo RPM Input**         | PWM Input (DMA) | —                          | High-speed turbine RPM capture |
| **Analog Pressure Sensors** | SPI         | [ADS7606 16-bit ADC](https://www.ti.com/product/ADS7606) | 8-channel, high-resolution |
| **Analog Temp Sensor**      | SPI         | ADS7606                      | 2-wire automotive NTC |
| **Thermocouples (x4)**      | SPI         | [MAX31856](https://www.adafruit.com/product/3263) ×4    | Cold junction compensated |
| **USB (Development only)**  | USB         | USB6B1RL                     | Routed to D+/D− pads on MicroMod |
| **FRAM (Optional)**         | SPI         | —                            | For logging or configuration |
| **SPI Bus**                 | SPI1 or SPI2 | Shared w/ chip select lines | All SPI devices use dedicated CS |

### Power

- **12V input** powers the system.
- **5V regulated output** feeds sensors.
- All sensors share a common system ground (GND).

### MicroMod / Teensy Notes

- **MicroMod Teensy** is used for final installations. USB routed via ESD protection.
- **Teensy 4.1** is used for development using the onboard micro-USB port.
- All SPI devices have dedicated chip-select lines.
- PWM and RPM input pins are chosen to ensure DMA support and avoid bus conflicts.

---

## Pin Usage

The following table details how each Teensy pin is used across both Teensy 4.1 and MicroMod Teensy:

| Function | Teensy Pin | Teensy 4.1 Label | TeensyMM Pin | MicroMod Label |
|----------|------------|------------------|--------------|----------------|
| Actuator PWM Out | 2 | PWM | 47 | PWM1 |
| Actuator CAN RX | 0 | CRX1 | 19 | UART_RX1 |
| Actuator CAN TX | 1 | CTX1 | 17 | UART_TX1 |
| J1939 CAN RX | 30 | CRX3 | 41 | CAN_RX |
| J1939 CAN TX | 31 | CTX3 | 43 | CAN_TX |
| FRAM CS | 32 | OUT1B | 65 | G9 |
| MOSI | 11 | MOSI | 59 | SPI_COPI |
| MISO | 12 | MISO | 61 | SPI_CIPO |
| SCK | 13 | SCK | 57 | SPI_SCK |
| Comp In CS | 10 | CS | 55 | SPI_CS |
| Comp Out CS | 40 | A16 | 40 | G0 |
| Turbine In CS | 41 | A17 | 42 | G1 |
| Turbine Out CS | 6 | OUT1D PWM | 71 | G6 |
| DB0 | 3 | LRCLK2 | 32 | PWM0 |
| DB1 | 4 | BCLK2 | 10 | D0 |
| DB2 | 5 | IN2 | 18 | D1 |
| DB3 | 7 | OUT1D | 56 | AUD_OUT |
| DB4 | 8 | TX2 | 54 | AUD_IN |
| DB5 | 9 | OUT1C | 69 | G7 |
| DB6 | 14 | A0 | 34 | A0 |
| DB7 | 15 | A1 | 38 | A1 |
| DB8 | 16 | A2 | 20 | UART_RX2 |
| DB9 | 17 | A3 | 22 | UART_TX2 |
| DB10 | 34 | RX8 | 66 | SDIO_DATA1 |
| DB11 | 35 | TX8 | 64 | SDIO_DATA0 |
| DB12 | 20 | A6 | 52 | AUD_LRCLK |
| DB13 | 21 | A7 | 50 | AUD_BLCK |
| DB14 | 23 | A9 | 58 | AUD_MCLK | 
| DB15 | 24 | A10 | 53 | I2C_SCL1 |
| RD | 25 | A11 | 51 | I2C_SDA1 |
| CS | 26 | A12 | 67 | G8 |
| RESET | 27 | A13 | 8 | G11 |
| CONVST_A/B | 29 | TX7 | 16 | I2C_INT |
| BUSY | 33 | MCLK2 | 63 | G10 |
| SDA | 18 | SDA | 12 | I2C_SDA |
| SCL | 19 | SCL | 14 | I2C_SCL |

---

## Building the Project

This project uses [PlatformIO](https://platformio.org/) as the build system.

### Steps to Build:
1. Install [PlatformIO IDE](https://platformio.org/install) for VSCode or CLI.
2. Clone this repository.
3. Open the project in PlatformIO.
4. Select your board (`Teensy 4.1` or `MicroMod Teensy`).
5. Click **Build** and **Upload**.

---

## Configuration

Configuration is done in source code. See `config.h` and `main.cpp` for available options and sensor/channel definitions.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.
