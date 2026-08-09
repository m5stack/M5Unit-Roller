/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "unit_roller_iap.hpp"

#include <string.h>

namespace {

constexpr uint8_t kWriteCommand          = 0x06;
constexpr uint8_t kStartCommand          = 0x77;
constexpr uint8_t kJumpRegister          = 0xFD;
constexpr uint8_t kLegacyCommandRegister = 20;
constexpr uint8_t kLegacyScratchRegister = 24;
constexpr uint8_t kLegacyJumpCommand     = 0xDE;
constexpr uint8_t kMotorIdRegister       = 0x10;
constexpr uint8_t kProductRegister       = 0xF4;
constexpr uint8_t kVersionRegister       = 0xFE;
constexpr uint8_t kAddressRegister       = 0xFF;
constexpr uint8_t kMaximumBootStatus     = 12;

}  // namespace

bool UnitRollerIAP::begin(TwoWire *wire, int sda, int scl, uint32_t speed)
{
    if (wire == nullptr) return false;
    wire_  = wire;
    sda_   = sda;
    scl_   = scl;
    speed_ = speed;
    return restartWire();
}

bool UnitRollerIAP::restartWire()
{
    if (wire_ == nullptr) return false;
    wire_->end();
    if (!wire_->begin(sda_, scl_)) return false;
    wire_->setClock(speed_);
    wire_->setTimeOut(500);
    return wire_->setBufferSize(8192);
}

void UnitRollerIAP::holdBusLow(uint32_t durationMs)
{
    wire_->end();
    pinMode(scl_, OUTPUT);
    pinMode(sda_, OUTPUT);
    digitalWrite(scl_, LOW);
    digitalWrite(sda_, LOW);
    delay(durationMs);
    restartWire();
}

bool UnitRollerIAP::probe(uint8_t address)
{
    wire_->beginTransmission(address);
    return wire_->endTransmission() == 0;
}

bool UnitRollerIAP::read(uint8_t address, uint8_t reg, void *data, size_t length)
{
    if (wire_ == nullptr || data == nullptr || length == 0 || length > 255) return false;
    wire_->beginTransmission(address);
    if (wire_->write(reg) != 1 || wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom(address, static_cast<uint8_t>(length)) != length) {
        while (wire_->available() > 0) wire_->read();
        return false;
    }
    auto *bytes = static_cast<uint8_t *>(data);
    for (size_t i = 0; i < length; ++i) {
        if (wire_->available() <= 0) return false;
        bytes[i] = static_cast<uint8_t>(wire_->read());
    }
    return true;
}

bool UnitRollerIAP::read8(uint8_t address, uint8_t reg, uint8_t &value)
{
    return read(address, reg, &value, sizeof(value));
}

bool UnitRollerIAP::write(uint8_t address, uint8_t reg, const void *data, size_t length)
{
    if (wire_ == nullptr || data == nullptr || length == 0 || length > 31) return false;
    wire_->beginTransmission(address);
    if (wire_->write(reg) != 1 || wire_->write(static_cast<const uint8_t *>(data), length) != length) {
        wire_->endTransmission();
        return false;
    }
    return wire_->endTransmission() == 0;
}

bool UnitRollerIAP::write8(uint8_t address, uint8_t reg, uint8_t value)
{
    return write(address, reg, &value, sizeof(value));
}

bool UnitRollerIAP::rawRead8(uint8_t address, uint8_t &value)
{
    if (wire_->requestFrom(address, static_cast<uint8_t>(1)) != 1 || wire_->available() <= 0) return false;
    value = static_cast<uint8_t>(wire_->read());
    return true;
}

bool UnitRollerIAP::rawWrite(uint8_t address, const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0) return false;
    wire_->beginTransmission(address);
    const size_t sent = wire_->write(data, length);
    return wire_->endTransmission() == 0 && sent == length;
}

bool UnitRollerIAP::readApplication(uint8_t address, UnitRollerIAPDevice &device)
{
    uint8_t addressFirst  = 0;
    uint8_t addressSecond = 0;
    uint8_t addressFinal  = 0;
    uint8_t productFirst  = 0;
    uint8_t productSecond = 0;
    uint8_t version       = 0;
    uint8_t motorId       = 0;

    // Priming with 0xFF makes an unsupported 0xF4 return the address on legacy firmware.
    if (!read8(address, kAddressRegister, addressFirst) || addressFirst != address ||
        !read8(address, kProductRegister, productFirst) || !read8(address, kAddressRegister, addressSecond) ||
        addressSecond != address || !read8(address, kProductRegister, productSecond) ||
        !read8(address, kVersionRegister, version) || version == 0 || version > 31 ||
        !read8(address, kMotorIdRegister, motorId) || !read8(address, kAddressRegister, addressFinal) ||
        addressFinal != address) {
        return false;
    }

    const bool productKnown =
        productFirst == productSecond &&
        ((productFirst == static_cast<uint8_t>(UnitRollerIAPProduct::Roller485) && version >= 2) ||
         (productFirst == static_cast<uint8_t>(UnitRollerIAPProduct::RollerCAN) && version >= 3));
    device.mode            = UnitRollerIAPMode::Application;
    device.address         = address;
    device.firmwareVersion = version;
    device.productKnown    = productKnown;
    device.productId       = productKnown ? productFirst : 0;
    device.motorId         = motorId;
    device.legacyDefaultId =
        !productKnown && version <= 2 && (motorId == kRoller485DefaultId || motorId == kRollerCANDefaultId);
    return true;
}

bool UnitRollerIAP::readBootloaderStatus(uint8_t &status)
{
    uint8_t first  = 0;
    uint8_t second = 0;
    if (!rawRead8(kBootloaderAddress, first) || first > kMaximumBootStatus) return false;
    delay(5);
    if (!rawRead8(kBootloaderAddress, second) || second > kMaximumBootStatus || first != second) return false;
    status = second;
    return true;
}

UnitRollerIAPDevice UnitRollerIAP::detect()
{
    UnitRollerIAPDevice found;
    if (wire_ == nullptr) return found;

    uint8_t candidates     = 0;
    uint8_t bootStatus     = 0;
    bool bootloaderPresent = false;
    for (uint8_t address = 0x08; address <= 0x77; ++address) {
        if (address == kBootloaderAddress) {
            UnitRollerIAPDevice candidate;
            if (readApplication(address, candidate)) {
                found = candidate;
                ++candidates;
            } else {
                bootloaderPresent = readBootloaderStatus(bootStatus);
            }
            continue;
        }
        if (!probe(address)) continue;
        UnitRollerIAPDevice candidate;
        if (readApplication(address, candidate)) {
            found = candidate;
            ++candidates;
        }
    }

    if (candidates + (bootloaderPresent ? 1 : 0) > 1) {
        found      = UnitRollerIAPDevice{};
        found.mode = UnitRollerIAPMode::Multiple;
        return found;
    }
    if (candidates == 1) return found;
    if (bootloaderPresent) {
        found.mode       = UnitRollerIAPMode::Bootloader;
        found.address    = kBootloaderAddress;
        found.bootStatus = bootStatus;
    }
    return found;
}

bool UnitRollerIAP::waitForAddress(uint8_t address, uint32_t timeoutMs)
{
    const uint32_t started = millis();
    do {
        if (probe(address)) return true;
        delay(50);
    } while (millis() - started < timeoutMs);
    return false;
}

bool UnitRollerIAP::enterBootloader(uint8_t applicationAddress)
{
    restartWire();
    write8(applicationAddress, kJumpRegister, 1);
    holdBusLow(200);
    if (waitForAddress(kBootloaderAddress, 5000)) return true;

    const uint8_t magic[4] = {0x55, 0xAA, 0x55, 0xAA};
    if (!write(applicationAddress, kLegacyScratchRegister, magic, sizeof(magic)) ||
        !write8(applicationAddress, kLegacyCommandRegister, kLegacyJumpCommand)) {
        return false;
    }
    holdBusLow(200);
    return waitForAddress(kBootloaderAddress, 10000);
}

bool UnitRollerIAP::writeFirmware(const uint8_t *image, uint32_t imageSize, UnitRollerIAPProgress progress,
                                  void *progressUser)
{
    if (image == nullptr || imageSize != kApplicationSize || !waitForAddress(kBootloaderAddress, 5000)) return false;

    uint32_t written = 0;
    if (progress != nullptr) progress(0, imageSize, progressUser);
    while (written < imageSize) {
        const uint16_t chunk =
            static_cast<uint16_t>((imageSize - written) > kPageSize ? kPageSize : (imageSize - written));
        const uint32_t target = kApplicationStart + written;
        packet_[0]            = kWriteCommand;
        packet_[1]            = static_cast<uint8_t>(target >> 24);
        packet_[2]            = static_cast<uint8_t>(target >> 16);
        packet_[3]            = static_cast<uint8_t>(target >> 8);
        packet_[4]            = static_cast<uint8_t>(target);
        packet_[5]            = static_cast<uint8_t>(chunk >> 8);
        packet_[6]            = static_cast<uint8_t>(chunk);
        packet_[7]            = 0xFF;
        memcpy(packet_ + 8, image + written, chunk);

        if (!waitForAddress(kBootloaderAddress, 30000) || !rawWrite(kBootloaderAddress, packet_, chunk + 8)) {
            return false;
        }
        written += chunk;
        if (progress != nullptr) progress(written, imageSize, progressUser);
        delay(80);
    }
    return true;
}

bool UnitRollerIAP::startApplication()
{
    if (!waitForAddress(kBootloaderAddress, 30000)) return false;
    const uint8_t command = kStartCommand;
    return rawWrite(kBootloaderAddress, &command, sizeof(command));
}

UnitRollerIAPResult UnitRollerIAP::update(const UnitRollerIAPDevice &device, UnitRollerIAPProduct product,
                                          uint8_t targetVersion, const uint8_t *image, uint32_t imageSize,
                                          bool allowUnverifiedProduct, UnitRollerIAPProgress progress,
                                          void *progressUser)
{
    UnitRollerIAPResult result;
    if (wire_ == nullptr || image == nullptr || imageSize != kApplicationSize) {
        result.error = UnitRollerIAPError::InvalidImage;
        return result;
    }
    if (device.mode == UnitRollerIAPMode::None) {
        result.error = UnitRollerIAPError::NoDevice;
        return result;
    }
    if (device.mode == UnitRollerIAPMode::Multiple) {
        result.error = UnitRollerIAPError::MultipleDevices;
        return result;
    }
    if (device.mode == UnitRollerIAPMode::Application) {
        if (device.productKnown && device.productId != static_cast<uint8_t>(product)) {
            result.error = UnitRollerIAPError::ProductMismatch;
            return result;
        }
        if (!device.productKnown && !allowUnverifiedProduct) {
            result.error = UnitRollerIAPError::ConfirmationRequired;
            return result;
        }
        if (device.productKnown && device.firmwareVersion > targetVersion) {
            result.error = UnitRollerIAPError::NewerVersion;
            return result;
        }
        if (device.productKnown && device.firmwareVersion == targetVersion) {
            result.error = UnitRollerIAPError::UpToDate;
            return result;
        }
        if (!enterBootloader(device.address)) {
            result.error = UnitRollerIAPError::EnterBootloader;
            return result;
        }
    } else if (device.mode == UnitRollerIAPMode::Bootloader) {
        if (!allowUnverifiedProduct) {
            result.error = UnitRollerIAPError::ConfirmationRequired;
            return result;
        }
        if (!waitForAddress(kBootloaderAddress, 3000)) {
            result.error = UnitRollerIAPError::BootloaderLost;
            return result;
        }
    }

    if (!writeFirmware(image, imageSize, progress, progressUser)) {
        result.error = UnitRollerIAPError::PageWrite;
        return result;
    }
    if (!startApplication()) {
        result.error = UnitRollerIAPError::StartApplication;
        return result;
    }

    const uint32_t started = millis();
    do {
        delay(200);
        UnitRollerIAPDevice verified = detect();
        if (verified.mode != UnitRollerIAPMode::Application) continue;
        result.applicationAddress = verified.address;
        if (!verified.productKnown || verified.productId != static_cast<uint8_t>(product)) {
            result.error = UnitRollerIAPError::VerifyProduct;
            return result;
        }
        if (verified.firmwareVersion != targetVersion) {
            result.error = UnitRollerIAPError::VerifyVersion;
            return result;
        }
        return result;
    } while (millis() - started < 12000);

    result.error = UnitRollerIAPError::VerifyTimeout;
    return result;
}

const char *UnitRollerIAP::errorText(UnitRollerIAPError error)
{
    switch (error) {
        case UnitRollerIAPError::InvalidImage:
            return "Invalid firmware image";
        case UnitRollerIAPError::NoDevice:
            return "No Unit Roller found";
        case UnitRollerIAPError::MultipleDevices:
            return "Connect only one Unit Roller";
        case UnitRollerIAPError::ProductMismatch:
            return "Wrong Unit Roller model";
        case UnitRollerIAPError::NewerVersion:
            return "Installed firmware is newer";
        case UnitRollerIAPError::UpToDate:
            return "Firmware is already up to date";
        case UnitRollerIAPError::ConfirmationRequired:
            return "Product confirmation required";
        case UnitRollerIAPError::EnterBootloader:
            return "Cannot enter Bootloader";
        case UnitRollerIAPError::BootloaderLost:
            return "Bootloader not responding";
        case UnitRollerIAPError::PageWrite:
            return "Firmware write failed";
        case UnitRollerIAPError::StartApplication:
            return "Cannot start application";
        case UnitRollerIAPError::VerifyTimeout:
            return "Application did not start";
        case UnitRollerIAPError::VerifyProduct:
            return "Product verification failed";
        case UnitRollerIAPError::VerifyVersion:
            return "Version verification failed";
        default:
            return "No error";
    }
}

const char *UnitRollerIAP::productName(UnitRollerIAPProduct product)
{
    return product == UnitRollerIAPProduct::Roller485 ? "Roller485" : "RollerCAN";
}
