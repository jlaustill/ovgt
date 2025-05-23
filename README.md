# OVGT - Open Source VGT Controller

An open-source Variable Geometry Turbo controller for Teensy 4.1 and MicroMod Teensy platforms.

## Hardware Requirements

### Supported Microcontrollers
- Teensy 4.1 (primary development target)
- MicroMod Teensy (secondary target)

### Required Components
- 20x4 I2C LCD Display
- CAN transceiver compatible with 3.3V logic

## Pin Usage

### Teensy 4.1
| Function | Pin | Description |
|----------|-----|-------------|
| Actuator Control | PWM Pin 2 | PWM output at 300Hz for VGT actuator control |
| CAN Bus | CAN2 | Communication with engine/vehicle CAN bus |
| LCD Display | I2C (pins 18/19) | Communication with 20x4 LCD display (address 0x27) |

### MicroMod Teensy
Same pin configuration as Teensy 4.1 (pin compatibility maintained across platforms).

## Building the Project

This project uses PlatformIO as the build system. To build:

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository
3. Open the project in PlatformIO
4. Build and upload to your Teensy board

## Configuration

See source code for configuration options.
