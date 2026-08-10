/*
 *SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <unit_roller485.hpp>

constexpr uint8_t kMotorId = 0x00;
UnitRoller485 roller;
HardwareSerial rollerSerial(2);

void setup()
{
    Serial.begin(115200);
    roller.begin(&rollerSerial, 115200, SERIAL_8N1, 13, 15);

    uint8_t uid[12]      = {};
    uint8_t deviceId     = 0;
    uint8_t firmware     = 0;
    int8_t uidResult     = roller.getUID(kMotorId, uid);
    int8_t idResult      = roller.getDeviceTypeID(kMotorId, &deviceId);
    int8_t versionResult = roller.getFirmwareVersion(kMotorId, &firmware);

    Serial.printf("device=0x%02X firmware=%u uid=", deviceId, firmware);
    for (uint8_t value : uid) {
        Serial.printf("%02X", value);
    }
    Serial.printf(" result=%d/%d/%d\n", uidResult, idResult, versionResult);
}

void loop()
{
    delay(1000);
}
