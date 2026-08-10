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
    roller.setOutput(kMotorId, false);
    roller.setMode(kMotorId, ROLLER_MODE_POSITION);
    roller.setPosTurnAngle(kMotorId, 1, 9000, 500);
}

void loop()
{
    int32_t targetTurn   = 0;
    uint16_t targetAngle = 0;
    int32_t actualTurn   = 0;
    uint16_t actualAngle = 0;
    int8_t targetResult  = roller.getPosTurnAngle(kMotorId, &targetTurn, &targetAngle);
    int8_t actualResult  = roller.getPosReadbackTurnAngle(kMotorId, &actualTurn, &actualAngle);
    Serial.printf("target=%ld+%.2f actual=%ld+%.2f result=%d/%d\n", static_cast<long>(targetTurn), targetAngle / 100.0f,
                  static_cast<long>(actualTurn), actualAngle / 100.0f, targetResult, actualResult);
    delay(1000);
}
