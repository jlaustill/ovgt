#ifndef compressorEfficiency_h
#define compressorEfficiency_h

// Isentropic (adiabatic) compressor efficiency from the pressure ratio and the
// measured inlet/outlet air temperatures:
//
//   e = ideal temp rise / actual temp rise
//     = Ti * (Pr^0.286 - 1) / (To - Ti)      [Ti, To absolute / Kelvin]
//
// Returns efficiency as a fraction (0..1), or a NEGATIVE sentinel (-1) when it is
// undefined — i.e. no real compression (BR = prRatio <= 1.0) or the measured rise
// is ~0 (the 0/0 case). citC / cotC are in Celsius (converted to Kelvin
// internally); prRatio = Po / Pi (= boost ratio, BR).
// NOTE: at low BR the outlet-temp rise is dominated by heat-soak of the COT probe
// rather than compression, so CE is only a true compressor efficiency at sustained
// boost; near idle it is computable but not physically meaningful.
float compressorEfficiency(float prRatio, float citC, float cotC);

#endif
