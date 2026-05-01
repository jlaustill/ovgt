# OVGT - Open Source VGT Controller

![PlatformIO](https://img.shields.io/badge/platformio-teensy-orange)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

An open-source Variable Geometry Turbo controller for Teensy 4.1.

## Hardware Requirements

### Microcontroller
- Teensy 4.1

### Required Components
- [SN65HVD232DR CAN Transceiver](https://www.ti.com/product/SN65HVD232) x2 (3.3V compatible)
- TIP120 Darlington transistor (VGT actuator PWM driver)

### Optional Components
- 20x4 I2C LCD Display (for debugging or status output)
- [FRAM Memory](https://www.cypress.com/products/serial-fram) (for non-volatile config/logging, SPI)
- ADS1115 x2 (16-bit ADC, I2C, for analog sensors)
- MAX31856 x4 (thermocouple amplifier, SPI)

## System Overview

This project uses a Teensy 4.1 for real-time embedded control of a Variable Geometry Turbocharger. The system communicates with the VGT actuator over a dedicated CAN bus, controls vane position via PWM output, and monitors the J1939 vehicle bus on a second CAN channel.

### Component Breakdown

| Function                    | Interface   | IC/Component                  | Notes |
|-----------------------------|-------------|-------------------------------|-------|
| **CAN - Actuator**          | CAN via UART (CRX3/CTX3) | SN65HVD232DR         | VGT actuator communication |
| **CAN - J1939**             | CAN via UART (CRX1/CTX1) | SN65HVD232DR         | Vehicle bus (future) |
| **PWM Output**              | PWM Pin     | TIP120                        | For VGT actuator control |
| **LCD Display (Optional)**  | I2C         | 20x4 I2C LCD                  | Status/debug display |
| **FRAM (Optional)**         | SPI         | ---                            | For logging or configuration |
| **ADS1115 #1 (Optional)**   | I2C (0x48)  | ADS1115                       | Analog sensors (ADDR→GND) |
| **ADS1115 #2 (Optional)**   | I2C (0x49)  | ADS1115                       | Analog sensors (ADDR→VDD) |
| **MAX31856 #1 (Optional)**  | SPI         | MAX31856                      | Thermocouple input |
| **MAX31856 #2 (Optional)**  | SPI         | MAX31856                      | Thermocouple input |
| **MAX31856 #3 (Optional)**  | SPI         | MAX31856                      | Thermocouple input |
| **MAX31856 #4 (Optional)**  | SPI         | MAX31856                      | Thermocouple input |

### Power

- **12V input** powers the system.
- **5V regulated output** feeds sensors.
- All components share a common system ground (GND).

---

## Pin Usage

| Function | Teensy Pin | Teensy 4.1 Label |
|----------|------------|------------------|
| J1939 CAN RX (future) | 0 | CRX1 |
| J1939 CAN TX (future) | 1 | CTX1 |
| Actuator PWM Out | 2 | PWM |
| MAX31856 #3 CS | 5 | - |
| MAX31856 #3 DRDY | 3 | - |
| MAX31856 #3 FLT | 4 | - |
| MAX31856 #4 CS | 26 | - |
| MAX31856 #4 DRDY | 24 | - |
| MAX31856 #4 FLT | 25 | - |
| FRAM HOLD | 8 | - |
| FRAM WP (Write Protect) | 9 | - |
| FRAM CS | 10 | CS |
| MOSI (FRAM SI / MAX31856 SDI) | 11 | MOSI |
| MISO (FRAM SO / MAX31856 SDO) | 12 | MISO |
| SCK (FRAM / MAX31856) | 13 | SCK |
| SDA (ADS1115 / LCD) | 18 | SDA |
| SCL (ADS1115 / LCD) | 19 | SCL |
| ADS1115 #2 ALERT/RDY (I2C addr 0x49) | 21 | - |
| Actuator CAN RX | 30 | CRX3 |
| Actuator CAN TX | 31 | CTX3 |
| 5V PSU Power Good | 33 | MCLK2 |
| MAX31856 #1 CS | 37 | - |
| MAX31856 #1 DRDY | 35 | - |
| MAX31856 #1 FLT | 36 | - |
| MAX31856 #2 CS | 40 | - |
| MAX31856 #2 DRDY | 38 | - |
| MAX31856 #2 FLT | 39 | - |
| ADS1115 #1 ALERT/RDY (I2C addr 0x48) | 41 | - |

---

## Sensor Channels

### ADS1115 #1 (I2C 0x48 — ADDR→GND)

| Channel | Sensor | Type | Notes |
|---------|--------|------|-------|
| A0 | COP — Compressor Output Pressure | 0.5–4.5V pressure transducer | 10 bar MAP |
| A1 | CIP — Compressor Input Pressure | 0.5–4.5V pressure transducer | 1 bar / 15 PSI |
| A2 | CIT — Compressor Input Temperature | GM NTC thermistor, 2.2kΩ pulldown | Atmospheric turbo only; use MAX31856 for primary |
| A3 | TIP — Turbine Input Pressure | 0.5–4.5V pressure transducer | 100 PSIG, zero-offset calibrated at startup |

### ADS1115 #2 (I2C 0x49 — ADDR→VDD)

| Channel | Sensor | Type | Notes |
|---------|--------|------|-------|
| A0 | TOP — Turbine Out Pressure | 0.5–4.5V pressure transducer | 30 PSI default — verify sensor rating |
| A1 | Oil Pressure | 0.5–4.5V pressure transducer | 100 PSI |
| A2 | Oil Temp | GM NTC thermistor, 2.2kΩ pulldown | Same family as CIT |
| A3 | Fuel Lift Pump Pressure (temporary) | 0.5–4.5V pressure transducer | 15 PSI |

### MAX31856 Thermocouple Channels

| IC | Sensor | Notes |
|----|--------|-------|
| MAX31856 #1 | COT — Compressor Output Temperature | |
| MAX31856 #2 | CIT — Compressor Input Temperature | Primary turbo only; use ADS1115 #1 A2 for atmospheric |
| MAX31856 #3 | TIT — Turbine Input Temperature | |
| MAX31856 #4 | TOT — Turbine Output Temperature | |

---

## Building the Project

This project uses [PlatformIO](https://platformio.org/) as the build system.

### Steps to Build:
1. Install [PlatformIO IDE](https://platformio.org/install) for VSCode or CLI.
2. Clone this repository.
3. Open the project in PlatformIO.
4. Click **Build** and **Upload**.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.
