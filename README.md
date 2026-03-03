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
| MOSI | 11 | MOSI |
| MISO | 12 | MISO |
| SCK | 13 | SCK |
| SDA | 18 | SDA |
| SCL | 19 | SCL |
| Actuator CAN RX | 30 | CRX3 |
| Actuator CAN TX | 31 | CTX3 |
| FRAM CS | 32 | OUT1B |

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
