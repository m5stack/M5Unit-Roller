/*
 *SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <Wire.h>
#include <unit_roller_iap.hpp>

#include "firmware_image.h"

constexpr int kSdaPin                         = 21;
constexpr int kSclPin                         = 22;
constexpr uint32_t kI2cFrequency              = 400000;
constexpr uint8_t kTargetVersion              = 3;
constexpr UnitRollerIAPProduct kTargetProduct = UnitRollerIAPProduct::RollerCAN;
constexpr char kConfirmation[]                = "UPDATE CAN";

UnitRollerIAP updater;

void showProgress(uint32_t written, uint32_t total, void *)
{
    static uint8_t previous = 255;
    const uint8_t percent   = total == 0 ? 0 : static_cast<uint8_t>((written * 100UL) / total);
    if (percent != previous && (percent % 5 == 0 || percent == 100)) {
        Serial.printf("Progress: %u%% (%lu/%lu bytes)\n", percent, static_cast<unsigned long>(written),
                      static_cast<unsigned long>(total));
        previous = percent;
    }
}

bool confirmUpdate()
{
    while (Serial.available() > 0) Serial.read();
    Serial.printf("Type '%s' and press Enter to start.\n", kConfirmation);
    for (;;) {
        if (Serial.available() <= 0) {
            delay(20);
            continue;
        }
        String response = Serial.readStringUntil('\n');
        response.trim();
        if (response == kConfirmation) return true;
        Serial.printf("Not confirmed. Type exactly: %s\n", kConfirmation);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(200);
    delay(1000);
    Serial.println("\nUnit RollerCAN firmware updater V3");
    Serial.println("Connect exactly one RollerCAN through I2C/GROVE.");
    Serial.println("Keep power connected until verification finishes.");

    if (!updater.begin(&Wire, kSdaPin, kSclPin, kI2cFrequency)) {
        Serial.println("ERROR: Cannot initialize I2C.");
        return;
    }

    Serial.println("Detecting Unit Roller...");
    const UnitRollerIAPDevice device = updater.detect();
    if (device.mode == UnitRollerIAPMode::None) {
        Serial.println("ERROR: No Unit Roller application or Bootloader found.");
        return;
    }
    if (device.mode == UnitRollerIAPMode::Multiple) {
        Serial.println("ERROR: Multiple Unit Roller devices found. Connect only one device.");
        return;
    }

    bool allowRiskyUpdate = false;
    if (device.mode == UnitRollerIAPMode::Application) {
        Serial.printf("Application found at 0x%02X, firmware V%u, communication ID 0x%02X.\n", device.address,
                      device.firmwareVersion, device.motorId);
        if (device.productKnown) {
            if (device.productId != static_cast<uint8_t>(kTargetProduct)) {
                allowRiskyUpdate = true;
                Serial.println("WARNING: Reported model is Roller485, but the selected image is RollerCAN.");
                Serial.println("Continue only to recover hardware that was previously flashed with the wrong image.");
            }
            if (device.firmwareVersion == kTargetVersion) {
                allowRiskyUpdate = true;
                Serial.println("WARNING: This will reflash the same firmware version.");
            }
            if (device.firmwareVersion > kTargetVersion) {
                allowRiskyUpdate = true;
                Serial.println("WARNING: This will downgrade the installed firmware.");
            }
        } else {
            allowRiskyUpdate = true;
            Serial.println("WARNING: Legacy firmware has no reliable product ID.");
            if (device.motorId == UnitRollerIAP::kRollerCANDefaultId) {
                Serial.println("The communication ID matches the RollerCAN factory default (0xA8).");
            } else if (device.motorId == UnitRollerIAP::kRoller485DefaultId) {
                Serial.println("CAUTION: The communication ID matches the Roller485 factory default (0x00).");
            } else {
                Serial.println("The communication ID was customized and cannot identify the product.");
            }
            Serial.println("Check that the connected product is RollerCAN before continuing.");
        }
    } else {
        allowRiskyUpdate = true;
        Serial.printf("Bootloader recovery mode found at 0x%02X.\n", device.address);
        Serial.println("The Bootloader contains no product identity. Confirm the product is RollerCAN.");
    }

    if (!confirmUpdate()) return;
    Serial.println("Starting update. Do not disconnect power or the I2C cable.");
    const UnitRollerIAPResult result = updater.update(device, kTargetProduct, kTargetVersion, appHexData, appHexLength,
                                                      allowRiskyUpdate, showProgress);
    if (!result.ok()) {
        Serial.printf("UPDATE FAILED: %s\n", UnitRollerIAP::errorText(result.error));
        return;
    }
    Serial.printf("UPDATE COMPLETE: RollerCAN V3 verified at I2C address 0x%02X.\n", result.applicationAddress);
}

void loop()
{
    delay(1000);
}
