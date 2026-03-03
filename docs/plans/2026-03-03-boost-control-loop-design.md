# Boost Control Loop Design

## Summary

Add closed-loop VGT vane control driven by a 10 bar MAP sensor read via an ADS1115 ADC over I2C. The system reads boost pressure, maps it to a vane position percentage through a tunable lookup table, and commands the actuator.

## Hardware

- ADS1115 16-bit ADC on I2C (address 0x48, shared bus with LCD on pins 18/19)
- 3.3V-5V level shifter on I2C lines
- 10 bar MAP sensor, 0.5-4.5V output, connected to ADS1115 channel A0
- ADS1115 gain: GAIN_TWOTHIRDS (+/-6.144V range)

## Architecture

Three modules with single responsibilities:

### BoostSensor (src/sensors/boostSensor.h/.cpp)
- Static class, same pattern as Actuator
- Reads ADS1115 channel A0 via Adafruit ADS1X15 library
- Converts ADC counts -> voltage -> absolute pressure in hPa
- Conversion: hPa = (voltage - 0.5) * 2500
- Range: 0.5V = 0 hPa, 4.5V = 10,000 hPa
- Writes result to AppData.boostPressureHpa

### BoostController (src/control/boostController.h/.cpp)
- Static class
- Reads AppData.boostPressureHpa
- Maps pressure to vane position (0-100%) via hardcoded lookup table
- Linearly interpolates between table entries
- Clamps to first/last entry outside table range
- Writes result to AppData.demandedVanePosition

Starting lookup table:

| Pressure (hPa) | Vane Position (%) |
|----------------|-------------------|
| <= 1000        | 21                |
| 1500           | 25                |
| 2000           | 35                |
| 3000           | 50                |

Table values are initial guesses to be tuned experimentally.

### Actuator (existing, modified)
- Remove hardcoded position variable `p`
- Read position from AppData.demandedVanePosition instead
- No other changes; continues to own PWM output and CAN receive

## Data Flow

```
ADS1115 (I2C) -> BoostSensor -> AppData.boostPressureHpa -> BoostController -> AppData.demandedVanePosition -> Actuator -> PWM
```

## AppData Changes

Add one field:
- `uint16_t boostPressureHpa` (0-10,000)

`demandedVanePosition` already exists, now written by BoostController instead of hardcoded.

## Main Loop Orchestration (ovgt.cpp)

```
BoostSensor::read()        // update pressure
BoostController::update()  // map pressure to position
Actuator::Loop()           // apply position to PWM (every 100ms)
```

BoostSensor and BoostController run every loop iteration. Actuator continues its existing 100ms update cycle.

## LCD Display

Add boost pressure to the display output. Existing demanded/reported position lines update automatically since they read from AppData.

## Dependencies

- Adafruit ADS1X15 library (add back to platformio.ini)
- Adafruit BusIO (required by ADS1X15)

## Future Considerations

- Upgrade from lookup table to PID control once hardware loop is validated
- Additional ADS1115 channels available (A1-A3) for future sensors
- Boost target could come from potentiometer or J1939 CAN instead of table
- FRAM storage for logging pressure/position data
