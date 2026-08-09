/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __UNIT_ROLLER_IAP_H
#define __UNIT_ROLLER_IAP_H

#include <Arduino.h>
#include <Wire.h>

enum class UnitRollerIAPProduct : uint8_t {
    Roller485 = 0x01,
    RollerCAN = 0x02,
};

enum class UnitRollerIAPMode : uint8_t {
    None,
    Application,
    Bootloader,
    Multiple,
};

struct UnitRollerIAPDevice {
    UnitRollerIAPMode mode  = UnitRollerIAPMode::None;
    uint8_t address         = 0;
    uint8_t firmwareVersion = 0;
    uint8_t productId       = 0;
    uint8_t motorId         = 0;
    uint8_t bootStatus      = 0;
    bool productKnown       = false;
    bool legacyDefaultId    = false;
};

enum class UnitRollerIAPError : uint8_t {
    None,
    InvalidImage,
    NoDevice,
    MultipleDevices,
    ProductMismatch,
    NewerVersion,
    UpToDate,
    ConfirmationRequired,
    EnterBootloader,
    BootloaderLost,
    PageWrite,
    StartApplication,
    VerifyTimeout,
    VerifyProduct,
    VerifyVersion,
};

struct UnitRollerIAPResult {
    UnitRollerIAPError error   = UnitRollerIAPError::None;
    uint8_t applicationAddress = 0;

    bool ok() const
    {
        return error == UnitRollerIAPError::None;
    }
};

using UnitRollerIAPProgress = void (*)(uint32_t written, uint32_t total, void *user);

class UnitRollerIAP {
public:
    static constexpr uint8_t kBootloaderAddress  = 0x54;
    static constexpr uint32_t kApplicationStart  = 0x08002000;
    static constexpr uint32_t kApplicationSize   = 0x1B800;
    static constexpr uint16_t kPageSize          = 0x0800;
    static constexpr uint8_t kRoller485DefaultId = 0x00;
    static constexpr uint8_t kRollerCANDefaultId = 0xA8;

    /** Initialize the ESP32 I2C bus and allocate the Bootloader transfer buffer. */
    bool begin(TwoWire *wire = &Wire, int sda = 21, int scl = 22, uint32_t speed = 400000);

    /** Find one Roller application or a Roller Bootloader on the I2C bus. */
    UnitRollerIAPDevice detect();

    /**
     * Write a complete application image and verify the resulting product and version.
     * Set allowUnverifiedProduct only after the user confirms a legacy or recovery-mode product.
     */
    UnitRollerIAPResult update(const UnitRollerIAPDevice &device, UnitRollerIAPProduct product, uint8_t targetVersion,
                               const uint8_t *image, uint32_t imageSize, bool allowUnverifiedProduct = false,
                               UnitRollerIAPProgress progress = nullptr, void *progressUser = nullptr);

    static const char *errorText(UnitRollerIAPError error);
    static const char *productName(UnitRollerIAPProduct product);

private:
    bool restartWire();
    void holdBusLow(uint32_t durationMs);
    bool probe(uint8_t address);
    bool read(uint8_t address, uint8_t reg, void *data, size_t length);
    bool read8(uint8_t address, uint8_t reg, uint8_t &value);
    bool write(uint8_t address, uint8_t reg, const void *data, size_t length);
    bool write8(uint8_t address, uint8_t reg, uint8_t value);
    bool rawRead8(uint8_t address, uint8_t &value);
    bool rawWrite(uint8_t address, const uint8_t *data, size_t length);
    bool readApplication(uint8_t address, UnitRollerIAPDevice &device);
    bool readBootloaderStatus(uint8_t &status);
    bool waitForAddress(uint8_t address, uint32_t timeoutMs);
    bool enterBootloader(uint8_t applicationAddress);
    bool writeFirmware(const uint8_t *image, uint32_t imageSize, UnitRollerIAPProgress progress, void *progressUser);
    bool startApplication();

    TwoWire *wire_                 = nullptr;
    int sda_                       = -1;
    int scl_                       = -1;
    uint32_t speed_                = 400000;
    uint8_t packet_[kPageSize + 8] = {};
};

#endif
