#ifndef cotThermistor_h
#define cotThermistor_h

// Convert the ADS divider voltage on the COT channel to °C for the RIFE Hi-AT
// NTC thermistor (1.0 kΩ pull-up to 5 V, sensor to GND). Returns false when the
// divider is railed (open/short) so the caller can hold its last good value.
bool cotThermistorReadC(float voltage, float *outTempC);

#endif
