#include <Arduino.h>
#include <unit_rollercan.hpp>

constexpr uint8_t kMotorId = 0xA8;
UnitRollerCAN roller;

void setup()
{
    Serial.begin(115200);
    roller.beginCAN(1000000, 16, 17);

    bool stallProtect    = false;
    bool posRangeProtect = false;
    bool keySwitch       = false;
    roller.getStallProtection(kMotorId, &stallProtect);
    roller.getPosRangeProtect(kMotorId, &posRangeProtect);
    roller.getKeySwitchMode(kMotorId, &keySwitch);

    // Write the current values back to demonstrate the R/W APIs without changing configuration.
    roller.setStallProtection(kMotorId, stallProtect);
    roller.setPosRangeProtect(kMotorId, posRangeProtect);
    roller.setKeySwitchMode(kMotorId, keySwitch);
    Serial.printf("stall=%u range=%u key=%u\n", stallProtect, posRangeProtect, keySwitch);
}

void loop()
{
    uint8_t status = 0;
    uint8_t error  = 0;
    roller.getSysStatus(kMotorId, &status);
    roller.getError(kMotorId, &error);
    Serial.printf("status=%u error=0x%02X\n", status, error);
    delay(1000);
}
