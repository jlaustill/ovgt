#pragma once
#include <Arduino.h>

class J1939 {
public:
    static void Initialize();
    static void Loop();

    // Unknown-PGN discovery inventory (main-context reads of ISR-updated table).
    static uint8_t unknownCount();
    static uint32_t unknownDroppedCount();
    static bool unknownAt(uint8_t i, uint32_t &pgn, uint8_t &sa, uint32_t &count,
                          uint32_t &firstMs, uint32_t &lastMs, uint8_t out8[8]);
};
