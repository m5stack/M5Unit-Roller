#include <Arduino.h>
#include <unit_rollercan.hpp>

constexpr uint8_t kMotorId = 0xA8;
UnitRollerCAN roller;

void setup()
{
    Serial.begin(115200);
    roller.beginCAN(1000000, 16, 17);

    uint8_t uid[12]       = {};
    uint8_t deviceId      = 0;
    uint8_t firmware      = 0;
    uint8_t canId         = 0;
    uint8_t commMode      = 0;
    uint8_t i2cAddr       = 0;
    roller_bps_t baudRate = ROLLER_BPS_CAN_1000000;
    roller.getUID(kMotorId, uid);
    roller.getDeviceTypeID(kMotorId, &deviceId);
    roller.getFirmwareVersion(kMotorId, &firmware);
    roller.getMotorID(kMotorId, &canId);
    roller.getBPS(kMotorId, &baudRate);
    roller.getCommunicationMode(kMotorId, &commMode);
    roller.getI2CAddress(kMotorId, &i2cAddr);

    Serial.printf("device=0x%02X firmware=%u can=0x%02X baud=%u comm=%u i2c=0x%02X uid=", deviceId, firmware, canId,
                  baudRate, commMode, i2cAddr);
    for (uint8_t value : uid) {
        Serial.printf("%02X", value);
    }
    Serial.println();
}

void loop()
{
    delay(1000);
}
