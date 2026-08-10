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
}

void loop()
{
    roller485_current_t current = {};
    roller485_config_t config   = {};
    int8_t currentResult        = roller.getCurrentParameters(kMotorId, &current);
    int8_t configResult         = roller.getTargetConfiguration(kMotorId, &config);

    Serial.printf("speed_limit=%.2f mA pos_limit=%.2f mA current_target=%.2f mA result=%d\n",
                  current.speedLimit / 100.0f, current.posLimit / 100.0f, current.currentTarget / 100.0f,
                  currentResult);
    Serial.printf(
        "speed_target=%.2f rpm pos_target=%.2f deg output=%u ov=%u stall=%u range=%u i2c=0x%02X baud=%u "
        "key=%u result=%d\n",
        config.speedTarget / 100.0f, config.posTarget / 100.0f, config.output, config.overVoltageRecovery,
        config.stallProtect, config.posRangeProtect, config.i2cAddr, config.baudRate, config.keySwitch, configResult);
    delay(1000);
}
