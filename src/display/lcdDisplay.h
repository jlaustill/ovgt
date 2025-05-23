#include "AppData.h"
#include <LiquidCrystal_I2C.h>

class LcdDisplay {
    public:
        LcdDisplay(uint8_t address, uint8_t cols, uint8_t rows);
        void init(AppData *appData);
        void updateDisplay(uint32_t loopCount);

    private:
        AppData *appData;
        LiquidCrystal_I2C lcd;
        char buffer[21];  // 20 chars + null terminator
};
