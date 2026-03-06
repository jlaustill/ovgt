#include "./lcdDisplay.h"

LcdDisplay::LcdDisplay(uint8_t address, uint8_t cols, uint8_t rows) : lcd(address, cols, rows) {
    this->lcd = LiquidCrystal_I2C(address, cols, rows);
}

void LcdDisplay::init() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void LcdDisplay::updateDisplay(uint32_t loopCount) {
    lcd.setCursor(0, 0);
    snprintf(buffer, 21, "Boost %5u hPa", appData.boostPressureHpa);
    lcd.print(buffer);

    lcd.setCursor(0, 1);
    snprintf(buffer, 21, "%2dC Load %5u S:%1d", appData.actuatorTemp, appData.actuatorMotorLoad, appData.actuatorStatus);
    lcd.print(buffer);

    lcd.setCursor(0, 2);
    snprintf(buffer, 21, "Demanded  %3d%%", appData.actuatorDemandedPosition);
    lcd.print(buffer);

    lcd.setCursor(0, 3);
    snprintf(buffer, 21, "Actuator Raw %4u", appData.actuatorRawPosition);
    lcd.print(buffer);
}
