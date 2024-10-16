/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_roller_common.hpp"

bool mutexLocked = false;
void acquireMutex()
{
    while (mutexLocked) {
        delay(1);
    }
    mutexLocked = true;
}

void releaseMutex()
{
    mutexLocked = false;
}

int32_t handleValue(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    int32_t newNumerical = 0;
    // Combine bytes to form the encoder value
    newNumerical |= ((int32_t)byte0);  // LSB
    newNumerical |= ((int32_t)byte1 << 8);
    newNumerical |= ((int32_t)byte2 << 16);
    newNumerical |= ((int32_t)byte3 << 24);  // MSB
    return newNumerical;
}