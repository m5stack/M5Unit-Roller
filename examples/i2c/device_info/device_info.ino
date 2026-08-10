/*
 *SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <unit_rolleri2c.hpp>

UnitRollerI2C roller;

void setup()
{
    Serial.begin(115200);
    if (!roller.begin(&Wire, I2C_ADDR, 21, 22, 400000)) {
        Serial.println("Roller I2C not found");
        return;
    }

    uint8_t uid[12] = {};
    roller.getUID(uid);
    Serial.printf("device=0x%02X firmware=%u uid=", roller.getDeviceTypeID(), roller.getFirmwareVersion());
    for (uint8_t value : uid) {
        Serial.printf("%02X", value);
    }
    Serial.println();
}

void loop()
{
    delay(1000);
}
