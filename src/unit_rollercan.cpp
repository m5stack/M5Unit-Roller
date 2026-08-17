/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_rollercan.hpp"

static twai_message_t ping_message = {
    .identifier = ROLLERCAN_MASTER, .data_length_code = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};

void UnitRollerCAN::beginCAN(unsigned long baud, int8_t rxPin, int8_t txPin)
{
    static twai_timing_config_t t_config;
    txNum         = txPin;
    rxNum         = rxPin;
    gpio_num_t tx = static_cast<gpio_num_t>(txPin);
    gpio_num_t rx = static_cast<gpio_num_t>(rxPin);
    switch (baud) {
        case 125000:
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case 500000:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 1000000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    // static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

int8_t UnitRollerCAN::setRemoveProtection(uint8_t id, bool en)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    uint8_t cmd_id;
    if (en) {
        cmd_id = ROLLERCAN_PROTECTION_ON;
    } else {
        cmd_id = ROLLERCAN_PROTECTION_OFF;
    }
    if (!sendData(id, cmd_id, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setUnprotect(uint8_t id)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_UNPROTECT, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setSaveFlash(uint8_t id)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_FLASH, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setBPS(uint8_t id, roller_bps_t bps)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_BAUDRATE, bps, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveCommandResponse(id, ROLLERCAN_BAUDRATE, bps, 0);
    if (result != ROLLER_WRITE_SUCCESS) {
        releaseMutex();
        return result;
    }
    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    // Map the new baud rate to the timing config
    switch (bps) {
        case 0:
            beginCAN(1000000, rxNum, txNum);
            break;
        case 1:
            beginCAN(500000, rxNum, txNum);
            break;
        case 2:
            beginCAN(125000, rxNum, txNum);
            break;
        default:
            beginCAN(1000000, rxNum, txNum);
            break;
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setOutput(uint8_t id, bool en)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    uint8_t cmd_id;
    if (en) {
        cmd_id = ROLLERCAN_ON;
    } else {
        cmd_id = ROLLERCAN_OFF;
    }
    if (!sendData(id, cmd_id, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (en != getOutput(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::getOutput(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SWITCH);
}

int8_t UnitRollerCAN::setMode(uint8_t id, roller_mode_t mode)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (ROLLERCAN_MODE & 0xFF);
    canData[1] = (ROLLERCAN_MODE >> 8) & 0xFF;
    canData[4] = mode;
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != getMode(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::getMode(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_MODE);
}

int8_t UnitRollerCAN::getSysStatus(uint8_t id)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));

    if (!sendData(id, ROLLERCAN_STATUS_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    if (result == ROLLER_WRITE_SUCCESS) {
        releaseMutex();
        return recv_status;
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::getError(uint8_t id)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_STATUS_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    if (result == ROLLER_WRITE_SUCCESS) {
        releaseMutex();
        return recv_faultInfo;
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setMotorID(uint8_t id, uint8_t newId)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_CMD_ID, newId, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    unsigned long started = millis();
    int8_t result         = ROLLER_SERIAL_TIMEOUT;
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }
        uint8_t command     = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t responseId  = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t broadcastId = static_cast<uint8_t>(frame.identifier);
        if (frame.extd && !frame.rtr && frame.data_length_code == 8 && command == 0 && responseId == newId &&
            broadcastId == 0xFE) {
            result = ROLLER_WRITE_SUCCESS;
            break;
        }
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setRGBMode(uint8_t id, roller_rgb_t mode)
{
    int8_t result = setMotorParameter(id, ROLLERCAN_RGB_MODE, mode);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != getRGBMode(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setRGB(uint8_t id, int32_t color)
{
    acquireMutex();
    clearPendingResponses();
    uint8_t rgbValues[3];
    memset(canData, 0, sizeof(canData));
    rgbValues[0] = (color >> 11) & 0x1F;  // R (5 bits)
    rgbValues[1] = (color >> 5) & 0x3F;   // G (6 bits)
    rgbValues[2] = color & 0x1F;          // B (5 bits)
    canData[0]   = (ROLLERCAN_RGB_COLOR & 0xFF);
    canData[1]   = (ROLLERCAN_RGB_COLOR >> 8) & 0xFF;
    canData[4]   = (rgbValues[2] * 255) / 31;  // Expand to 255
    canData[5]   = (rgbValues[1] * 255) / 63;  // Expand to 255
    canData[6]   = (rgbValues[0] * 255) / 31;  // Expand to 255
    const int32_t expectedColor =
        (static_cast<int32_t>(canData[6]) << 16) | (static_cast<int32_t>(canData[5]) << 8) | canData[4];
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return getMotorParameter(id, ROLLERCAN_RGB_COLOR) == expectedColor ? ROLLER_WRITE_SUCCESS : ROLLER_WRITE_FAILED;
}

int8_t UnitRollerCAN::setRGBBrightness(uint8_t id, uint8_t brightness)
{
    int8_t result = setMotorParameter(id, ROLLERCAN_RGB_BRIGHTNESS, brightness);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (brightness != getRGBBrightness(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setSpeed(uint8_t id, int32_t speed)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    speed                    = constrain(speed, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_SPEED, speed * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speed != getSpeed(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getSpeed(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_SPEED, 100);
}

int8_t UnitRollerCAN::setSpeedCurrent(uint8_t id, int32_t speedCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    speedCurrent             = constrain(speedCurrent, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_SPEED_CURRENT, speedCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedCurrent != getSpeedCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getSpeedCurrent(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_SPEED_CURRENT, 100);
}

int8_t UnitRollerCAN::setSpeedKp(uint8_t id, uint32_t speedKp)
{
    int8_t result = setMotorPID(id, ROLLERCAN_SPEED_KP, speedKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKp != getSpeedKp(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getSpeedKp(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SPEED_KP);
}

int8_t UnitRollerCAN::setSpeedKi(uint8_t id, uint32_t speedKi)
{
    int8_t result = setMotorPID(id, ROLLERCAN_SPEED_KI, speedKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKi != getSpeedKi(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getSpeedKi(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SPEED_KI);
}

int8_t UnitRollerCAN::setSpeedKd(uint8_t id, uint32_t speedKd)
{
    int8_t result = setMotorPID(id, ROLLERCAN_SPEED_KD, speedKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKd != getSpeedKd(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getSpeedKd(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SPEED_KD);
}

int8_t UnitRollerCAN::setPosition(uint8_t id, int32_t position)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    position                 = constrain(position, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_POSITION, position * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (position != getPosition(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getPosition(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_POSITION, 100);
}

int8_t UnitRollerCAN::setPositionCurrent(uint8_t id, int32_t positionCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    positionCurrent          = constrain(positionCurrent, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_POSITION_CURRENT, positionCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionCurrent != getPositionCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getPositionCurrent(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_POSITION_CURRENT, 100);
}

int8_t UnitRollerCAN::setPositionKp(uint8_t id, uint32_t opsitionKp)
{
    int8_t result = setMotorPID(id, ROLLERCAN_POSITION_KP, opsitionKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (opsitionKp != getPositionKp(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getPositionKp(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_POSITION_KP);
}

int8_t UnitRollerCAN::setPositionKi(uint8_t id, uint32_t positionKi)
{
    int8_t result = setMotorPID(id, ROLLERCAN_POSITION_KI, positionKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKi != getPositionKi(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getPositionKi(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_POSITION_KI);
}

int8_t UnitRollerCAN::setPositionKd(uint8_t id, uint32_t positionKd)
{
    int8_t result = setMotorPID(id, ROLLERCAN_POSITION_KD, positionKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKd != getPositionKd(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::getPositionKd(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_POSITION_KD);
}

int8_t UnitRollerCAN::setCurrent(uint8_t id, int32_t current)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    current                  = constrain(current, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_CURRENT, current * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (current != getCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getCurrent(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_CURRENT, 100);
}

int8_t UnitRollerCAN::setEncoder(uint8_t id, int32_t encoder)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    encoder                  = constrain(encoder, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_ENCODER, encoder);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (encoder != getEncoder(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getEncoder(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_ENCODER);
}

int32_t UnitRollerCAN::getActualSpeed(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_ACTUAL_SPEED, 100);
}

int32_t UnitRollerCAN::getActualPosition(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_ACTUAL_POSITION, 100);
}

int32_t UnitRollerCAN::getActualCurrent(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_ACTUAL_CURRENT, 100);
}

int32_t UnitRollerCAN::getActualVin(uint8_t id)
{
    return getScaledMotorParameter(id, ROLLERCAN_VIN, 100);
}

int32_t UnitRollerCAN::getActualTemp(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_TEMP);
}

int8_t UnitRollerCAN::getRGBMode(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_RGB_MODE);
}

int8_t UnitRollerCAN::getRGB(uint8_t id, uint8_t *rgbValues)
{
    if (rgbValues == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    int8_t result = getMotorParameter(id, ROLLERCAN_RGB_COLOR);
    rgbValues[0]  = readData[6];
    rgbValues[1]  = readData[5];
    rgbValues[2]  = readData[4];
    return result;
}

int8_t UnitRollerCAN::getRGBBrightness(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_RGB_BRIGHTNESS);
}

int8_t UnitRollerCAN::writeRemoveProtection(uint8_t id, uint8_t addr, bool en)
{
    return writeMotorParameter(id, addr, ROLLERCAN_PROTECTION, en);
}

int8_t UnitRollerCAN::writeSaveFlash(uint8_t id, uint8_t addr, bool en)
{
    return writeMotorParameter(id, addr, ROLLERCAN_SAVE_FLASH, en);
}

int8_t UnitRollerCAN::writeOutput(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_SWITCH, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (en != readOutput(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readOutput(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SWITCH);
}

int8_t UnitRollerCAN::writeMode(uint8_t id, uint8_t addr, roller_mode_t mode)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_MODE, mode);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != readMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readMode(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_MODE);
}

int8_t UnitRollerCAN::writeSpeed(uint8_t id, uint8_t addr, int32_t speed)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_SPEED, speed * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speed != readSpeed(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readSpeed(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_SPEED, 100);
}

int8_t UnitRollerCAN::writeSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    speedCurrent             = constrain(speedCurrent, min, max);
    int8_t result            = writeMotorParameter(id, addr, ROLLERCAN_SPEED_CURRENT, speedCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedCurrent != readSpeedCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readSpeedCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_SPEED_CURRENT, 100);
}

int8_t UnitRollerCAN::writeSpeedKp(uint8_t id, uint8_t addr, uint32_t speedKp)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_SPEED_KP, speedKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKp != readSpeedKp(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readSpeedKp(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SPEED_KP);
}

int8_t UnitRollerCAN::writeSpeedKi(uint8_t id, uint8_t addr, uint32_t speedKi)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_SPEED_KI, speedKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKi != readSpeedKi(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readSpeedKi(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SPEED_KI);
}

int8_t UnitRollerCAN::writeSpeedKd(uint8_t id, uint8_t addr, uint32_t speedKd)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_SPEED_KD, speedKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKd != readSpeedKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readSpeedKd(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SPEED_KD);
}

int8_t UnitRollerCAN::writePosition(uint8_t id, uint8_t addr, int32_t position)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_POSITION, position * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (position != readPosition(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readPosition(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_POSITION, 100);
}

int8_t UnitRollerCAN::writePositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    positionCurrent          = constrain(positionCurrent, min, max);
    int8_t result            = writeMotorParameter(id, addr, ROLLERCAN_POSITION_CURRENT, positionCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionCurrent != readPositionCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readPositionCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_POSITION_CURRENT, 100);
}

int8_t UnitRollerCAN::writePositionKp(uint8_t id, uint8_t addr, uint32_t positionKp)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_POSITION_KP, positionKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKp != readPositionKp(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readPositionKp(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_POSITION_KP);
}

int8_t UnitRollerCAN::writePositionKi(uint8_t id, uint8_t addr, uint32_t positionKi)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_POSITION_KI, positionKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKi != readPositionKi(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readPositionKi(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_POSITION_KI);
}

int8_t UnitRollerCAN::writePositionKd(uint8_t id, uint8_t addr, uint32_t positionKd)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_POSITION_KD, positionKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKd != readPositionKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readPositionKd(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_POSITION_KD);
}

int8_t UnitRollerCAN::writeCurrent(uint8_t id, uint8_t addr, int32_t current)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    current                  = constrain(current, min, max);
    int8_t result            = writeMotorParameter(id, addr, ROLLERCAN_CURRENT, current * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (current != readCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_CURRENT, 100);
}

int8_t UnitRollerCAN::writeEncoder(uint8_t id, uint8_t addr, int32_t encoder)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_ENCODER, encoder);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (encoder != readEncoder(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readEncoder(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_ENCODER);
}

int32_t UnitRollerCAN::readActualSpeed(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_ACTUAL_SPEED, 100);
}

int32_t UnitRollerCAN::readActualPosition(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_ACTUAL_POSITION, 100);
}

int32_t UnitRollerCAN::readActualCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_ACTUAL_CURRENT, 100);
}

int32_t UnitRollerCAN::readActualVin(uint8_t id, uint8_t addr)
{
    return readScaledMotorParameter(id, addr, ROLLERCAN_VIN, 100);
}

int32_t UnitRollerCAN::readActualTemp(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_TEMP);
}

int8_t UnitRollerCAN::readRGBMode(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_RGB_MODE);
}

int8_t UnitRollerCAN::writeRGBMode(uint8_t id, uint8_t addr, roller_rgb_t mode)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_RGB_MODE, mode);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != readRGBMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues)
{
    if (rgbValues == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    int32_t value = readMotorParameter(id, addr, ROLLERCAN_RGB_COLOR);
    int8_t result = value < 0 ? static_cast<int8_t>(value) : ROLLER_WRITE_SUCCESS;
    rgbValues[0]  = static_cast<uint8_t>(value >> 16);
    rgbValues[1]  = static_cast<uint8_t>(value >> 8);
    rgbValues[2]  = static_cast<uint8_t>(value);
    return result;
}

int8_t UnitRollerCAN::writeRGB(uint8_t id, uint8_t addr, int32_t color)
{
    acquireMutex();
    clearPendingResponses();
    uint8_t rgbValues[3];
    memset(canData, 0, sizeof(canData));
    canData[0]   = (ROLLERCAN_RGB_COLOR & 0xFF);
    canData[1]   = (ROLLERCAN_RGB_COLOR >> 8) & 0xFF;
    canData[2]   = addr;
    rgbValues[0] = (color >> 11) & 0x1F;       // R (5 bits)
    rgbValues[1] = (color >> 5) & 0x3F;        // G (6 bits)
    rgbValues[2] = color & 0x1F;               // B (5 bits)
    canData[4]   = (rgbValues[2] * 255) / 31;  // Expand to 255
    canData[5]   = (rgbValues[1] * 255) / 63;  // Expand to 255
    canData[6]   = (rgbValues[0] * 255) / 31;
    if (!sendData(id, ROLLERCAN_WRITE_I2C_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    readRGB(id, addr, rgbValues);
    int32_t rgbInt = ((rgbValues[0] << 16) | (rgbValues[1] << 8) | rgbValues[2]);
    if (color != rgbInt) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRGBBrightness(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_RGB_BRIGHTNESS);
}

int8_t UnitRollerCAN::writeRGBBrightness(uint8_t id, uint8_t addr, uint8_t brightness)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_RGB_BRIGHTNESS, brightness);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (brightness != readRGBBrightness(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawOutput(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_OUTPUT_REG);
}

int8_t UnitRollerCAN::writeRawOutput(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_OUTPUT_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (en != readRawOutput(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawMode(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_MODE_REG);
}

int8_t UnitRollerCAN::writeRawMode(uint8_t id, uint8_t addr, roller_mode_t mode)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_MODE_REG, 1, 1, mode);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != readRawMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawSpeed(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_SPEED_REG, 100);
}

int8_t UnitRollerCAN::writeRawSpeed(uint8_t id, uint8_t addr, int32_t speed)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_REG, 1, 4, speed * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speed != readRawSpeed(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawSpeedCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_SPEED_MAX_CURRENT_REG, 100);
}

int8_t UnitRollerCAN::writeRawSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_MAX_CURRENT_REG, 1, 4, speedCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedCurrent != readRawSpeedCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawSpeedKp(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_PID_REG);
}

int8_t UnitRollerCAN::writeRawSpeedKp(uint8_t id, uint8_t addr, uint32_t speedKp)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_PID_REG, 1, 4, speedKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKp != readRawSpeedKp(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawSpeedKi(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_PID_REG + 4);
}

int8_t UnitRollerCAN::writeRawSpeedKi(uint8_t id, uint8_t addr, uint32_t speedKi)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_PID_REG + 4, 1, 4, speedKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKi != readRawSpeedKi(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawSpeedKd(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_PID_REG + 8);
}

int8_t UnitRollerCAN::writeRawSpeedKd(uint8_t id, uint8_t addr, uint32_t speedKd)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_PID_REG + 8, 1, 4, speedKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (speedKd != readRawSpeedKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawPosition(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_POS_REG, 100);
}

int8_t UnitRollerCAN::writeRawPosition(uint8_t id, uint8_t addr, int32_t position)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_REG, 1, 4, position * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (position != readRawPosition(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawPositionCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_POS_MAX_CURRENT_REG, 100);
}

int8_t UnitRollerCAN::writeRawPositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_MAX_CURRENT_REG, 1, 4, positionCurrent * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionCurrent != readRawPositionCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawPositionKp(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_PID_REG);
}

int8_t UnitRollerCAN::writeRawPositionKp(uint8_t id, uint8_t addr, uint32_t positionKp)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_PID_REG, 1, 4, positionKp);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKp != readRawPositionKp(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawPositionKi(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_PID_REG + 4);
}

int8_t UnitRollerCAN::writeRawPositionKi(uint8_t id, uint8_t addr, uint32_t positionKi)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_PID_REG + 4, 1, 4, positionKi);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKi != readRawPositionKi(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

uint32_t UnitRollerCAN::readRawPositionKd(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_PID_REG + 8);
}

int8_t UnitRollerCAN::writeRawPositionKd(uint8_t id, uint8_t addr, uint32_t positionKd)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_PID_REG + 8, 1, 4, positionKd);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (positionKd != readRawPositionKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_CURRENT_REG, 100);
}

int8_t UnitRollerCAN::writeRawCurrent(uint8_t id, uint8_t addr, int32_t current)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_CURRENT_REG, 1, 4, current * 100);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (current != readRawCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawEncoder(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_DIAL_COUNTER_REG);
}

int8_t UnitRollerCAN::writeRawEncoder(uint8_t id, uint8_t addr, int32_t encoder)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_DIAL_COUNTER_REG, 1, 4, encoder);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (encoder != readRawEncoder(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeRawFlash(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SAVE_FLASH_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawActualSpeed(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_SPEED_READBACK_REG, 100);
}

int32_t UnitRollerCAN::readRawActualPosition(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_POS_READBACK_REG, 100);
}

int32_t UnitRollerCAN::readRawActualCurrent(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_CURRENT_READBACK_REG, 100);
}

int32_t UnitRollerCAN::readRawActualVin(uint8_t id, uint8_t addr)
{
    return readScaledMotorRawParameter(id, addr, I2C_VIN_REG, 100);
}

int32_t UnitRollerCAN::readRawActualTemp(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_TEMP_REG);
}

int8_t UnitRollerCAN::readRawRGBMode(uint8_t id, uint8_t addr)
{
    int32_t returnValue = readMotorRawParameter(id, addr, I2C_RGB_REG);
    if (returnValue < 0) {
        return returnValue;
    }
    return readData[3];
}

int8_t UnitRollerCAN::writeRawRGBMode(uint8_t id, uint8_t addr, roller_rgb_t mode)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_RGB_REG + 3, 1, 1, mode);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (mode != readRawRGBMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues)
{
    if (rgbValues == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    int32_t value = readMotorRawParameter(id, addr, I2C_RGB_REG);
    int8_t result = value < 0 ? static_cast<int8_t>(value) : ROLLER_WRITE_SUCCESS;
    rgbValues[0]  = readData[2];  // R value
    rgbValues[1]  = readData[1];  // G value
    rgbValues[2]  = readData[0];  // B value
    return result;
}

int8_t UnitRollerCAN::writeRawRGB(uint8_t id, uint8_t addr, int32_t color)
{
    acquireMutex();
    clearPendingResponses();
    uint8_t rgbValues[3];
    memset(canData, 0, sizeof(canData));
    rgbValues[0] = (color >> 11) & 0x1F;  // R (5 bits)
    rgbValues[1] = (color >> 5) & 0x3F;   // G (6 bits)
    rgbValues[2] = color & 0x1F;          // B (5 bits)
    uint8_t new_value;

    canData[0] = 4;
    canData[1] = I2C_RGB_REG;
    canData[2] = (rgbValues[2] * 255) / 31;  // Expand to 255
    canData[3] = (rgbValues[1] * 255) / 63;  // Expand to 255
    canData[4] = (rgbValues[0] * 255) / 31;
    if (!sendData(id, ROLLERCAN_WRITE_I2C_RAW_CMD, addr | 0x80, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveCommandResponse(id, ROLLERCAN_WRITE_I2C_RAW_CMD, static_cast<uint8_t>(addr | 0x81), 0);
    releaseMutex();
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    readRawRGB(id, addr, rgbValues);
    int32_t rgbInt = ((rgbValues[0] << 16) | (rgbValues[1] << 8) | rgbValues[2]);
    if (color != rgbInt) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawRGBBrightness(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_RGB_BRIGHTNESS_REG);
}

int8_t UnitRollerCAN::writeRawRGBBrightness(uint8_t id, uint8_t addr, uint8_t brightness)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_RGB_BRIGHTNESS_REG, 1, 1, brightness);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (brightness != readRawRGBBrightness(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawFirmwareVersion(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_FIRMWARE_VERSION_REG);
}

uint8_t UnitRollerCAN::readRawI2CAddress(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_ADDRESS_REG);
}

int8_t UnitRollerCAN::writeRawI2CAddress(uint8_t id, uint8_t addr, uint8_t newAddr)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_ADDRESS_REG, 1, 1, newAddr);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (newAddr != readRawI2CAddress(id, newAddr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeResetStallProtect(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_RESET_STALLED_PROTECT_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readStallProtect(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_STALL_PROTECTION_REG);
}

int8_t UnitRollerCAN::writePosRangeProtect(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_RANGE_PROTECT_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeStallProtect(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_STALL_PROTECTION_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (en != readStallProtect(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readButton(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_KEY_SWTICH_MODE_REG);
}

int8_t UnitRollerCAN::writeButton(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_KEY_SWTICH_MODE_REG, 1, 1, en);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (en != readButton(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readErrorCode(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_ERROR_CODE_REG);
}

int8_t UnitRollerCAN::readSysStatus(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SYS_STATUS_REG);
}

uint8_t UnitRollerCAN::readMotorID(uint8_t id, uint8_t addr)
{
    readMotorRawParameter(id, addr, I2C_ID_REG);
    return readData[3];
}

int8_t UnitRollerCAN::writeMotorID(uint8_t id, uint8_t addr, uint8_t motorId)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_ID_REG, 1, 1, motorId);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (motorId != readMotorID(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readBPS(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_BPS_REG);
}

int8_t UnitRollerCAN::writeBPS(uint8_t id, uint8_t addr, roller_bps_t bps)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_BPS_REG, 1, 1, bps);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    if (bps != readBPS(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setPosTurnAngle(uint8_t id, int32_t turn, uint16_t angle, int32_t maxCurrent)
{
    if (angle > 35999U || maxCurrent < 0 || maxCurrent > 1200) {
        return ROLLER_WRITE_FAILED;
    }

    uint8_t request[8]  = {};
    uint8_t response[8] = {};
    writeInt32LE(request, turn);
    request[4] = static_cast<uint8_t>(angle);
    request[5] = static_cast<uint8_t>(angle >> 8);
    request[6] = static_cast<uint8_t>(maxCurrent);
    request[7] = static_cast<uint8_t>(maxCurrent >> 8);

    int8_t result = transactCommand(id, ROLLERCAN_EXTENDED_POSITION_CMD, 0, request, response);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return memcmp(request, response, sizeof(request)) == 0 ? ROLLER_WRITE_SUCCESS : ROLLER_UNEXPECTED_RESPONSE;
}

int8_t UnitRollerCAN::getPosTurnAngle(uint8_t id, int32_t *turn, uint16_t *angle, uint16_t *maxCurrent)
{
    if (turn == nullptr || angle == nullptr || maxCurrent == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    uint8_t request[8]  = {};
    uint8_t response[8] = {};
    int8_t result       = transactCommand(id, ROLLERCAN_READ_EXTENDED_POSITION_CMD, 0, request, response);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }

    uint16_t angleValue   = readUint16LE(response + 4);
    uint16_t currentValue = readUint16LE(response + 6);
    if (angleValue > 35999U || currentValue > 1200U) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    *turn       = readInt32LE(response);
    *angle      = angleValue;
    *maxCurrent = currentValue;
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::getPosReadbackTurnAngle(uint8_t id, int32_t *turn, uint16_t *angle, int16_t *current)
{
    if (turn == nullptr || angle == nullptr || current == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    uint8_t request[8]  = {};
    uint8_t response[8] = {};
    int8_t result       = transactCommand(id, ROLLERCAN_READ_EXTENDED_POSITION_CMD, 1, request, response);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }

    uint16_t angleValue = readUint16LE(response + 4);
    if (angleValue > 35999U) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    *turn    = readInt32LE(response);
    *angle   = angleValue;
    *current = static_cast<int16_t>(readUint16LE(response + 6));
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::getFirmwareVersion(uint8_t id, uint8_t *version)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_FIRMWARE_VERSION, &value);
    if (result == ROLLER_WRITE_SUCCESS && version != nullptr && value >= 0 && value <= UINT8_MAX) {
        *version = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getDeviceTypeID(uint8_t id, uint8_t *deviceId)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_PRODUCT_DEVICE_ID, &value);
    if (result == ROLLER_WRITE_SUCCESS && deviceId != nullptr && value >= 0 && value <= UINT8_MAX) {
        *deviceId = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getUID(uint8_t id, uint8_t *uid)
{
    if (uid == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }
    int32_t words[3] = {};
    for (uint16_t index = 0; index < 3; ++index) {
        int8_t result = readFunctionValue(id, ROLLERCAN_UID_WORD_0 + index, &words[index]);
        if (result != ROLLER_WRITE_SUCCESS) {
            return result;
        }
    }
    memcpy(uid, words, sizeof(words));
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::getMotorID(uint8_t id, uint8_t *motorId)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_CAN_ID, &value);
    if (result == ROLLER_WRITE_SUCCESS && motorId != nullptr && value >= 0 && value <= UINT8_MAX) {
        *motorId = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getBPS(uint8_t id, roller_bps_t *bps)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_CAN_BAUD_INDEX, &value);
    if (result == ROLLER_WRITE_SUCCESS && bps != nullptr && value >= 0 && value <= 2) {
        *bps = static_cast<roller_bps_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getCommunicationMode(uint8_t id, uint8_t *mode)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_COMMUNICATION_MODE, &value);
    if (result == ROLLER_WRITE_SUCCESS && mode != nullptr && value >= 1 && value <= 3) {
        *mode = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getI2CAddress(uint8_t id, uint8_t *address)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_I2C_ADDRESS, &value);
    if (result == ROLLER_WRITE_SUCCESS && address != nullptr && value >= 0x08 && value <= 0x77) {
        *address = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getSysStatus(uint8_t id, uint8_t *status)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_SYSTEM_STATUS, &value);
    if (result == ROLLER_WRITE_SUCCESS && status != nullptr && value >= 0 && value <= 2) {
        *status = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::getError(uint8_t id, uint8_t *error)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_ERROR_CODE, &value);
    if (result == ROLLER_WRITE_SUCCESS && error != nullptr && value >= 0 && value <= UINT8_MAX) {
        *error = static_cast<uint8_t>(value);
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::setStallProtection(uint8_t id, bool en)
{
    int8_t result = writeFunctionValue(id, ROLLERCAN_STALL_PROTECTION, en ? 1 : 0);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    bool readback = false;
    result        = getStallProtection(id, &readback);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return readback == en ? ROLLER_WRITE_SUCCESS : ROLLER_UNEXPECTED_RESPONSE;
}

int8_t UnitRollerCAN::getStallProtection(uint8_t id, bool *en)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_STALL_PROTECTION, &value);
    if (result == ROLLER_WRITE_SUCCESS && en != nullptr && (value == 0 || value == 1)) {
        *en = value != 0;
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::setPosRangeProtect(uint8_t id, bool en)
{
    int8_t result = writeFunctionValue(id, ROLLERCAN_POSITION_RANGE_PROTECTION, en ? 1 : 0);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    bool readback = false;
    result        = getPosRangeProtect(id, &readback);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return readback == en ? ROLLER_WRITE_SUCCESS : ROLLER_UNEXPECTED_RESPONSE;
}

int8_t UnitRollerCAN::getPosRangeProtect(uint8_t id, bool *en)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_POSITION_RANGE_PROTECTION, &value);
    if (result == ROLLER_WRITE_SUCCESS && en != nullptr && (value == 0 || value == 1)) {
        *en = value != 0;
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::setKeySwitchMode(uint8_t id, bool en)
{
    int8_t result = writeFunctionValue(id, ROLLERCAN_BUTTON_MODE_SWITCH, en ? 1 : 0);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    bool readback = false;
    result        = getKeySwitchMode(id, &readback);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    return readback == en ? ROLLER_WRITE_SUCCESS : ROLLER_UNEXPECTED_RESPONSE;
}

int8_t UnitRollerCAN::getKeySwitchMode(uint8_t id, bool *en)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_BUTTON_MODE_SWITCH, &value);
    if (result == ROLLER_WRITE_SUCCESS && en != nullptr && (value == 0 || value == 1)) {
        *en = value != 0;
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::startAngleCal(uint8_t id)
{
    return writeFunctionValue(id, ROLLERCAN_START_ANGLE_CAL, 1);
}

int8_t UnitRollerCAN::updateAngleCal(uint8_t id)
{
    return writeFunctionValue(id, ROLLERCAN_UPDATE_ANGLE_CAL, 1);
}

int8_t UnitRollerCAN::getCalBusyStatus(uint8_t id, bool *busy)
{
    int32_t value = 0;
    int8_t result = readFunctionValue(id, ROLLERCAN_ANGLE_CAL_BUSY, &value);
    if (result == ROLLER_WRITE_SUCCESS && busy != nullptr && (value == 0 || value == 1)) {
        *busy = value != 0;
        return ROLLER_WRITE_SUCCESS;
    }
    return result == ROLLER_WRITE_SUCCESS ? ROLLER_UNEXPECTED_RESPONSE : result;
}

int8_t UnitRollerCAN::transactCommand(uint8_t id, uint8_t command, uint8_t commandParameter, const uint8_t *request,
                                      uint8_t *response)
{
    if (request == nullptr || response == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    acquireMutex();
    twai_message_t stale = {};
    while (twai_receive(&stale, 0) == ESP_OK) {
    }
    if (!sendData(id, command, commandParameter, request)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }

    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }
        uint8_t responseCommand = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t responseResult  = static_cast<uint8_t>(frame.identifier >> 16);
        uint8_t hostId          = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t motorId         = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || responseCommand != command ||
            responseResult != 1 || hostId != 0 || motorId != id) {
            continue;
        }
        memcpy(response, frame.data, 8);
        releaseMutex();
        return ROLLER_WRITE_SUCCESS;
    }
    releaseMutex();
    return ROLLER_SERIAL_TIMEOUT;
}

int8_t UnitRollerCAN::readFunctionValue(uint8_t id, uint16_t functionIndex, int32_t *value)
{
    if (value == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    uint8_t request[8] = {};
    request[0]         = static_cast<uint8_t>(functionIndex);
    request[1]         = static_cast<uint8_t>(functionIndex >> 8);

    acquireMutex();
    twai_message_t stale = {};
    while (twai_receive(&stale, 0) == ESP_OK) {
    }
    if (!sendData(id, ROLLERCAN_READ_CMD, 0, request)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }

    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }
        uint8_t command = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t hostId  = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t motorId = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || command != ROLLERCAN_READ_CMD || hostId != 0 ||
            motorId != id || readUint16LE(frame.data) != functionIndex || frame.data[2] != 0 || frame.data[3] != 0) {
            continue;
        }
        *value = readInt32LE(frame.data + 4);
        releaseMutex();
        return ROLLER_WRITE_SUCCESS;
    }
    releaseMutex();
    return ROLLER_SERIAL_TIMEOUT;
}

int8_t UnitRollerCAN::writeFunctionValue(uint8_t id, uint16_t functionIndex, int32_t value)
{
    uint8_t request[8] = {};
    request[0]         = static_cast<uint8_t>(functionIndex);
    request[1]         = static_cast<uint8_t>(functionIndex >> 8);
    writeInt32LE(request + 4, value);

    acquireMutex();
    twai_message_t stale = {};
    while (twai_receive(&stale, 0) == ESP_OK) {
    }
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, request)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }

    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }
        uint8_t command = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t motorId = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t hostId  = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || command != ROLLERCAN_STATUS_CMD ||
            motorId != id || hostId != 0) {
            continue;
        }
        releaseMutex();
        return ROLLER_WRITE_SUCCESS;
    }
    releaseMutex();
    return ROLLER_SERIAL_TIMEOUT;
}

int32_t UnitRollerCAN::readInt32LE(const uint8_t *data)
{
    return static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
}

uint16_t UnitRollerCAN::readUint16LE(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

void UnitRollerCAN::writeInt32LE(uint8_t *data, int32_t value)
{
    uint32_t raw = static_cast<uint32_t>(value);
    data[0]      = static_cast<uint8_t>(raw);
    data[1]      = static_cast<uint8_t>(raw >> 8);
    data[2]      = static_cast<uint8_t>(raw >> 16);
    data[3]      = static_cast<uint8_t>(raw >> 24);
}

bool UnitRollerCAN::sendData(uint8_t can_id, uint8_t cmd_id, uint16_t option, const uint8_t *data)
{
    ping_message.flags |= (1 << 0);
    ping_message.identifier = 0x00000000 | (cmd_id << 24) | (option << 16) | can_id;
    for (int i = 0; i < 8; i++) {
        ping_message.data[i] = data[i];
    }
#if defined UNIT_ROLLER_DEBUG
    Serial.printf("send data: 0x%02X ", ping_message.identifier);
    for (int i = 0; i < 8; i++) {
        Serial.printf("0x%02X ", ping_message.data[i]);
    }
    Serial.printf("\n");
#else
#endif
    esp_err_t transmitResult = twai_transmit(&ping_message, 2000);

    if (transmitResult == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

int8_t UnitRollerCAN::receiveData()
{
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(300)) == ESP_OK) {
#if defined UNIT_ROLLER_DEBUG
        Serial.printf("received identifier: 0x%02X\r\n", rx_msg.identifier);
        Serial.print("received data: ");
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            Serial.printf("0x%02X ", rx_msg.data[i]);
        }
        Serial.println("\r\n");
#else
#endif
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            readData[i] = rx_msg.data[i];
        }
        // Extract the CAN ID and other information
        recv_motorId   = (rx_msg.identifier >> 8) & 0xFF;   // Bit8~Bit15: CAN ID of current motor
        recv_faultInfo = (rx_msg.identifier >> 16) & 0x07;  // Bit16~18: Fault information
        recv_mode      = (rx_msg.identifier >> 19) & 0x07;  // Bit19~21: mode
        recv_status    = (rx_msg.identifier >> 22) & 0x03;  // Bit22~23: status
        return ROLLER_WRITE_SUCCESS;
    }
    return ROLLER_UNEXPECTED_RESPONSE;
}

void UnitRollerCAN::clearPendingResponses()
{
    twai_message_t stale = {};
    while (twai_receive(&stale, 0) == ESP_OK) {
    }
}

int8_t UnitRollerCAN::receiveStatusResponse(uint8_t id)
{
    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }

        uint8_t command = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t motorId = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t hostId  = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || command != ROLLERCAN_STATUS_CMD ||
            motorId != id || hostId != 0) {
            continue;
        }

        recv_motorId   = motorId;
        recv_faultInfo = static_cast<uint8_t>((frame.identifier >> 16) & 0x07);
        recv_mode      = static_cast<uint8_t>((frame.identifier >> 19) & 0x07);
        recv_status    = static_cast<uint8_t>((frame.identifier >> 22) & 0x03);
        memcpy(readData, frame.data, sizeof(readData));
        return ROLLER_WRITE_SUCCESS;
    }
    return ROLLER_SERIAL_TIMEOUT;
}

int8_t UnitRollerCAN::receiveCommandResponse(uint8_t id, uint8_t command, uint8_t expectedCommandParameter,
                                             uint8_t expectedHostId, uint8_t *response)
{
    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }

        uint8_t responseCommand  = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t commandParameter = static_cast<uint8_t>(frame.identifier >> 16);
        uint8_t hostId           = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t motorId          = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || responseCommand != command ||
            commandParameter != expectedCommandParameter || hostId != expectedHostId || motorId != id) {
            continue;
        }

        if (response != nullptr) {
            memcpy(response, frame.data, 8);
        }
        return ROLLER_WRITE_SUCCESS;
    }
    return ROLLER_SERIAL_TIMEOUT;
}

int8_t UnitRollerCAN::receiveParameterResponse(uint8_t id, uint8_t command, uint16_t parameterId, int32_t *value)
{
    if (value == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    unsigned long started = millis();
    while (millis() - started < 300) {
        twai_message_t frame = {};
        if (twai_receive(&frame, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }

        uint8_t responseCommand  = static_cast<uint8_t>((frame.identifier >> 24) & 0x1F);
        uint8_t commandParameter = static_cast<uint8_t>(frame.identifier >> 16);
        uint8_t hostId           = static_cast<uint8_t>(frame.identifier >> 8);
        uint8_t motorId          = static_cast<uint8_t>(frame.identifier);
        if (!frame.extd || frame.rtr || frame.data_length_code != 8 || responseCommand != command || hostId != 0 ||
            motorId != id || commandParameter != 0 || readUint16LE(frame.data) != parameterId || frame.data[2] != 0 ||
            frame.data[3] != 0) {
            continue;
        }

        *value = readInt32LE(frame.data + 4);
        memcpy(readData, frame.data, sizeof(readData));
        recv_motorId = motorId;
        return ROLLER_WRITE_SUCCESS;
    }
    return ROLLER_SERIAL_TIMEOUT;
}

int8_t UnitRollerCAN::setMotorParameter(uint8_t id, uint16_t parameterId, int32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    canData[4] = (parameterData & 0xFF);
    canData[5] = (parameterData >> 8) & 0xFF;
    canData[6] = (parameterData >> 16) & 0xFF;
    canData[7] = (parameterData >> 24) & 0xFF;
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setMotorPID(uint8_t id, uint16_t parameterId, uint32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    canData[4] = (parameterData & 0xFF);
    canData[5] = (parameterData >> 8) & 0xFF;
    canData[6] = (parameterData >> 16) & 0xFF;
    canData[7] = (parameterData >> 24) & 0xFF;
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::requestParameterValue(uint8_t id, uint8_t command, uint8_t address, uint16_t parameterId,
                                            int32_t *value)
{
    if (value == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    if (command == ROLLERCAN_READ_I2C_CMD) {
        canData[2] = address;
    }
    if (!sendData(id, command, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveParameterResponse(id, command, parameterId, value);
    releaseMutex();
    return result;
}

int32_t UnitRollerCAN::getMotorParameter(uint8_t id, uint16_t parameterId)
{
    int32_t value = 0;
    int8_t result = requestParameterValue(id, ROLLERCAN_READ_CMD, 0, parameterId, &value);
#if defined UNIT_ROLLER_DEBUG
    if (result == ROLLER_WRITE_SUCCESS) {
        Serial.printf("Value:%d\n", value);
    }
#else
#endif
    return result == ROLLER_WRITE_SUCCESS ? value : result;
}

int32_t UnitRollerCAN::getScaledMotorParameter(uint8_t id, uint16_t parameterId, int32_t divisor)
{
    int32_t value = 0;
    int8_t result = requestParameterValue(id, ROLLERCAN_READ_CMD, 0, parameterId, &value);
    return result == ROLLER_WRITE_SUCCESS ? value / divisor : result;
}

int32_t UnitRollerCAN::readMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId)
{
    int32_t value = 0;
    int8_t result = requestParameterValue(id, ROLLERCAN_READ_I2C_CMD, addr, parameterId, &value);
#if defined UNIT_ROLLER_DEBUG
    if (result == ROLLER_WRITE_SUCCESS) {
        Serial.printf("returnValue:%d\n", value);
    }
#else
#endif
    return result == ROLLER_WRITE_SUCCESS ? value : result;
}

int32_t UnitRollerCAN::readScaledMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId, int32_t divisor)
{
    int32_t value = 0;
    int8_t result = requestParameterValue(id, ROLLERCAN_READ_I2C_CMD, addr, parameterId, &value);
    return result == ROLLER_WRITE_SUCCESS ? value / divisor : result;
}

int32_t UnitRollerCAN::readScaledMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg, int32_t divisor)
{
    int32_t value = 0;
    int8_t result = requestRawParameterValue(id, addr, reg, &value);
    return result == ROLLER_WRITE_SUCCESS ? value / divisor : result;
}

int8_t UnitRollerCAN::writeMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId, int32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    canData[2] = addr;
    canData[4] = (parameterData & 0xFF);
    canData[5] = (parameterData >> 8) & 0xFF;
    canData[6] = (parameterData >> 16) & 0xFF;
    canData[7] = (parameterData >> 24) & 0xFF;
    if (!sendData(id, ROLLERCAN_WRITE_I2C_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}
int8_t UnitRollerCAN::writeMotorPID(uint8_t id, uint8_t addr, uint16_t parameterId, uint32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    canData[2] = addr;
    canData[4] = (parameterData & 0xFF);
    canData[5] = (parameterData >> 8) & 0xFF;
    canData[6] = (parameterData >> 16) & 0xFF;
    canData[7] = (parameterData >> 24) & 0xFF;
    if (!sendData(id, ROLLERCAN_WRITE_I2C_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveStatusResponse(id);
    releaseMutex();
    return result;
}

int32_t UnitRollerCAN::readMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg)
{
    int32_t value = 0;
    int8_t result = requestRawParameterValue(id, addr, reg, &value);
    return result == ROLLER_WRITE_SUCCESS ? value : result;
}

int8_t UnitRollerCAN::requestRawParameterValue(uint8_t id, uint8_t addr, uint8_t reg, int32_t *value)
{
    if (value == nullptr) {
        return ROLLER_UNEXPECTED_RESPONSE;
    }

    memset(canData, 0, sizeof(canData));
    int8_t result = writeMotorRawParameter(id, addr, reg, 1, 0, 0);
    if (result != ROLLER_WRITE_SUCCESS) {
        return result;
    }
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    canData[0] = addr;
    canData[1] = 4;
    if (!sendData(id, ROLLERCAN_READ_I2C_RAW_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    uint8_t response[8] = {};
    int8_t res          = receiveCommandResponse(id, ROLLERCAN_READ_I2C_RAW_CMD, 1, 0, response);
    if (res != ROLLER_WRITE_SUCCESS) {
        releaseMutex();
        return res;
    }
    memcpy(readData, response, sizeof(readData));
    *value = readInt32LE(response);
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len,
                                             int32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    uint8_t new_value;
    canData[0] = len + 1;
    canData[1] = reg;
    canData[2] = (parameterData & 0xFF);
    canData[3] = (parameterData >> 8) & 0xFF;
    canData[4] = (parameterData >> 16) & 0xFF;
    canData[5] = (parameterData >> 24) & 0xFF;
    if (option == 1) {
        new_value = addr | 0x80;  // If option is 1, set the high value
    } else {
        new_value = addr;
    }
    if (!sendData(id, ROLLERCAN_WRITE_I2C_RAW_CMD, new_value, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveCommandResponse(id, ROLLERCAN_WRITE_I2C_RAW_CMD, static_cast<uint8_t>(new_value | 0x01), 0);
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::writeMotorRawPID(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len,
                                       uint32_t parameterData)
{
    acquireMutex();
    clearPendingResponses();
    memset(canData, 0, sizeof(canData));
    uint8_t new_value;
    canData[0] = len + 1;
    canData[1] = reg;
    canData[2] = (parameterData & 0xFF);
    canData[3] = (parameterData >> 8) & 0xFF;
    canData[4] = (parameterData >> 16) & 0xFF;
    canData[5] = (parameterData >> 24) & 0xFF;
    if (option == 1) {
        new_value = addr | 0x80;  // If option is 1, set the high value
    } else {
        new_value = addr;
    }
    if (!sendData(id, ROLLERCAN_WRITE_I2C_RAW_CMD, new_value, canData)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;
    }
    int8_t result = receiveCommandResponse(id, ROLLERCAN_WRITE_I2C_RAW_CMD, static_cast<uint8_t>(new_value | 0x01), 0);
    releaseMutex();
    return result;
}
