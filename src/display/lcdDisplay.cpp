#include "./lcdDisplay.h"

LcdDisplay::LcdDisplay(uint8_t address, uint8_t cols, uint8_t rows) : lcd(address, cols, rows) {
    this->appData = nullptr;
    this->lcd = LiquidCrystal_I2C(address, cols, rows);
}

void LcdDisplay::init(AppData *appData) {
    this->appData = appData;
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void LcdDisplay::updateDisplay(uint32_t loopCount) {
        lcd.setCursor(0, 0);
        snprintf(buffer, 21, "%2d C Motor Load %3d", appData->actuatorTemp, appData->actuatorMotorLoad);
        lcd.print(buffer);

        lcd.setCursor(0, 1);
        snprintf(buffer, 21, "Status %1d L/S %d", appData->actuatorStatus, loopCount);
        lcd.print(buffer);
        
        lcd.setCursor(0, 2);
        snprintf(buffer, 21, "Demanded %d   ", (100 - appData->actuatorDemandedPosition));
        lcd.print(buffer);

        lcd.setCursor(0, 3);
        snprintf(buffer, 21, "Reported %d   ", appData->actuatorReportedPosition);
        lcd.print(buffer);
}