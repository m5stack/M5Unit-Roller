/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */
#include "unit_rolleri2c.hpp"
#include <M5Unified.h>

#define ROLLER_CALIBRATION_DELAY 10000

UnitRollerI2C RollerI2C;  // Create a UNIT_ROLLERI2C object

uint8_t is_roller_valid             = 0;
uint8_t is_roller_calibrated        = 0;
uint32_t roller_start_delay_counter = 0;

void setup()
{
    M5.begin();
    if (RollerI2C.begin(&Wire, 0x64, 21, 22, 400000)) {
        is_roller_valid            = 1;
        roller_start_delay_counter = millis();
    }
}

void loop()
{
    if (is_roller_valid) {
        if (millis() - roller_start_delay_counter < ROLLER_CALIBRATION_DELAY) {
            printf("Calibration will start after %dS\n",
                   (roller_start_delay_counter - (millis() - roller_start_delay_counter)) / 1000);
        } else {
            if (!is_roller_calibrated) {
                printf("Start encoder calibration\n");
                RollerI2C.setOutput(0);
                delay(100);
                RollerI2C.startAngleCal();
                delay(100);
                printf("Calibrationing...\n");
                while (RollerI2C.getCalBusyStatus()) {
                    printf("Calibrationing...\n");
                }
                RollerI2C.updateAngleCal();
                printf("Encoder calibration done\n");
                delay(500);
                RollerI2C.setOutput(0);
                RollerI2C.setMode(ROLLER_MODE_SPEED);
                RollerI2C.setSpeed(240000);
                RollerI2C.setSpeedMaxCurrent(100000);
                RollerI2C.setOutput(1);
                is_roller_calibrated = 1;
            }
        }
    } else {
        printf("No roller485 dectected\n");
    }
}
