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
    int8_t result = receiveData();
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setUnprotect(uint8_t id)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_UNPROTECT, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setSaveFlash(uint8_t id)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_FLASH, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setBPS(uint8_t id, roller_bps_t bps)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_BAUDRATE, bps, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
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
    receiveData();
    releaseMutex();
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
    memset(canData, 0, sizeof(canData));
    canData[0] = (ROLLERCAN_MODE & 0xFF);
    canData[1] = (ROLLERCAN_MODE >> 8) & 0xFF;
    canData[4] = mode;
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    receiveData();
    releaseMutex();
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
    memset(canData, 0, sizeof(canData));

    if (!sendData(id, ROLLERCAN_STATUS_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
    if (result) {
        releaseMutex();
        return recv_status;
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::getError(uint8_t id)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_STATUS_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
    if (result) {
        releaseMutex();
        return recv_faultInfo;
    }
    releaseMutex();
    return result;
}

int8_t UnitRollerCAN::setMotorID(uint8_t id, uint8_t newId)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    if (!sendData(id, ROLLERCAN_CMD_ID, newId, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setRGBMode(uint8_t id, roller_rgb_t mode)
{
    int8_t result = setMotorParameter(id, ROLLERCAN_RGB_MODE, mode);
    if (!result) {
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
    if (!sendData(id, ROLLERCAN_WRITE_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    receiveData();
    releaseMutex();
    getRGB(id, rgbValues);
    int32_t rgbInt = ((rgbValues[0] << 16) | (rgbValues[1] << 8) | rgbValues[2]);
    if (color != rgbInt) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setRGBBrightness(uint8_t id, uint8_t brightness)
{
    int8_t result = setMotorParameter(id, ROLLERCAN_RGB_BRIGHTNESS, brightness);
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (speed != getSpeed(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getSpeed(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SPEED) / 100.0f;
}

int8_t UnitRollerCAN::setSpeedCurrent(uint8_t id, int32_t speedCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    speedCurrent             = constrain(speedCurrent, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_SPEED_CURRENT, speedCurrent * 100);
    if (!result) {
        return result;
    }
    if (speedCurrent != getSpeedCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getSpeedCurrent(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_SPEED_CURRENT) / 100.0f;
}

int8_t UnitRollerCAN::setSpeedKp(uint8_t id, uint32_t speedKp)
{
    int8_t result = setMotorPID(id, ROLLERCAN_SPEED_KP, speedKp);
    if (!result) {
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
    if (!result) {
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
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (position != getPosition(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getPosition(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_POSITION) / 100.0f;
}

int8_t UnitRollerCAN::setPositionCurrent(uint8_t id, int32_t positionCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    positionCurrent          = constrain(positionCurrent, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_POSITION_CURRENT, positionCurrent * 100);
    if (!result) {
        return result;
    }
    if (positionCurrent != getPositionCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getPositionCurrent(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_POSITION_CURRENT) / 100.0f;
}

int8_t UnitRollerCAN::setPositionKp(uint8_t id, uint32_t opsitionKp)
{
    int8_t result = setMotorPID(id, ROLLERCAN_POSITION_KP, opsitionKp);
    if (!result) {
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
    if (!result) {
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
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (current != getCurrent(id)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getCurrent(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_CURRENT) / 100.0f;
}

int8_t UnitRollerCAN::setEncoder(uint8_t id, int32_t encoder)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    encoder                  = constrain(encoder, min, max);
    int8_t result            = setMotorParameter(id, ROLLERCAN_ENCODER, encoder);
    if (!result) {
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
    return getMotorParameter(id, ROLLERCAN_ACTUAL_SPEED) / 100.0f;
}

int32_t UnitRollerCAN::getActualPosition(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_ACTUAL_POSITION) / 100.0f;
}

int32_t UnitRollerCAN::getActualCurrent(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_ACTUAL_CURRENT) / 100.0f;
}

int32_t UnitRollerCAN::getActualVin(uint8_t id)
{
    return getMotorParameter(id, ROLLERCAN_VIN) / 100.0f;
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
    int32_t returnValue = getMotorParameter(id, ROLLERCAN_RGB_COLOR);
    rgbValues[0]        = readData[6];  // R value
    rgbValues[1]        = readData[5];  // G value
    rgbValues[2]        = readData[4];  // B value
    return returnValue;
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
    if (!result) {
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
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (speed != readSpeed(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readSpeed(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SPEED) / 100.0f;
}

int8_t UnitRollerCAN::writeSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    speedCurrent             = constrain(speedCurrent, min, max);
    int8_t result            = writeMotorParameter(id, addr, ROLLERCAN_SPEED_CURRENT, speedCurrent * 100);
    if (!result) {
        return result;
    }
    if (speedCurrent != readSpeedCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readSpeedCurrent(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_SPEED_CURRENT);
}

int8_t UnitRollerCAN::writeSpeedKp(uint8_t id, uint8_t addr, uint32_t speedKp)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_SPEED_KP, speedKp);
    if (!result) {
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
    if (!result) {
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
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (position != readPosition(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readPosition(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_POSITION) / 100.0f;
}

int8_t UnitRollerCAN::writePositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent)
{
    static const int32_t min = -1200;
    static const int32_t max = 1200;
    positionCurrent          = constrain(positionCurrent, min, max);
    int8_t result            = writeMotorParameter(id, addr, ROLLERCAN_POSITION_CURRENT, positionCurrent * 100);
    if (!result) {
        return result;
    }
    if (positionCurrent != readPositionCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readPositionCurrent(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_POSITION_CURRENT) / 100.0f;
}

int8_t UnitRollerCAN::writePositionKp(uint8_t id, uint8_t addr, uint32_t positionKp)
{
    int8_t result = writeMotorPID(id, addr, ROLLERCAN_POSITION_KP, positionKp);
    if (!result) {
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
    if (!result) {
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
    if (!result) {
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
    if (!result) {
        return result;
    }
    if (current != readCurrent(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readCurrent(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_CURRENT) / 100.0f;
}

int8_t UnitRollerCAN::writeEncoder(uint8_t id, uint8_t addr, int32_t encoder)
{
    int8_t result = writeMotorParameter(id, addr, ROLLERCAN_ENCODER, encoder);
    if (!result) {
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
    return readMotorParameter(id, addr, ROLLERCAN_ACTUAL_SPEED) / 100;
}

int32_t UnitRollerCAN::readActualPosition(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_ACTUAL_POSITION) / 100;
}

int32_t UnitRollerCAN::readActualCurrent(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_ACTUAL_CURRENT) / 100;
}

int32_t UnitRollerCAN::readActualVin(uint8_t id, uint8_t addr)
{
    return readMotorParameter(id, addr, ROLLERCAN_VIN) / 100;
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
    if (!result) {
        return result;
    }
    if (mode != readRGBMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues)
{
    int32_t returnValue = readMotorParameter(id, addr, ROLLERCAN_RGB_COLOR);
    rgbValues[0]        = readData[6];  // R value
    rgbValues[1]        = readData[5];  // G value
    rgbValues[2]        = readData[4];  // B value
    return returnValue;
}

int8_t UnitRollerCAN::writeRGB(uint8_t id, uint8_t addr, int32_t color)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
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
    if (!result) {
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
    if (result < 0) {
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
    if (result < 0) {
        return result;
    }
    if (mode != readRawMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawSpeed(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_REG) / 100.0f;
}

int8_t UnitRollerCAN::writeRawSpeed(uint8_t id, uint8_t addr, int32_t speed)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_REG, 1, 4, speed * 100);
    if (result < 0) {
        return result;
    }
    if (speed != readRawSpeed(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawSpeedCurrent(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_MAX_CURRENT_REG) / 100.0f;
}

int8_t UnitRollerCAN::writeRawSpeedCurrent(uint8_t id, uint8_t addr, int32_t speedCurrent)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_SPEED_MAX_CURRENT_REG, 1, 4, speedCurrent * 100);
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
        return result;
    }
    if (speedKd != readRawSpeedKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawPosition(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_REG) / 100.0f;
}

int8_t UnitRollerCAN::writeRawPosition(uint8_t id, uint8_t addr, int32_t position)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_REG, 1, 4, position * 100);
    if (result < 0) {
        return result;
    }
    if (position != readRawPosition(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawPositionCurrent(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_MAX_CURRENT_REG) / 100.0f;
}

int8_t UnitRollerCAN::writeRawPositionCurrent(uint8_t id, uint8_t addr, int32_t positionCurrent)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_POS_MAX_CURRENT_REG, 1, 4, positionCurrent * 100);
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
        return result;
    }
    if (positionKd != readRawPositionKd(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawCurrent(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_CURRENT_REG) / 100.0f;
}

int8_t UnitRollerCAN::writeRawCurrent(uint8_t id, uint8_t addr, int32_t current)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_CURRENT_REG, 1, 4, current * 100);
    if (result < 0) {
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
    if (result < 0) {
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
    if (!result) {
        return result;
    }
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readRawActualSpeed(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_SPEED_READBACK_REG) / 100.0f;
}

int32_t UnitRollerCAN::readRawActualPosition(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_POS_READBACK_REG) / 100.0f;
}

int32_t UnitRollerCAN::readRawActualCurrent(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_CURRENT_READBACK_REG) / 100.0f;
}

int32_t UnitRollerCAN::readRawActualVin(uint8_t id, uint8_t addr)
{
    return readMotorRawParameter(id, addr, I2C_VIN_REG) / 100.0f;
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
    if (result < 0) {
        return result;
    }
    if (mode != readRawRGBMode(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::readRawRGB(uint8_t id, uint8_t addr, uint8_t *rgbValues)
{
    int32_t returnValue = readMotorRawParameter(id, addr, I2C_RGB_REG);
    rgbValues[0]        = readData[2];  // R value
    rgbValues[1]        = readData[1];  // G value
    rgbValues[2]        = readData[0];  // B value

    return returnValue;
}

int8_t UnitRollerCAN::writeRawRGB(uint8_t id, uint8_t addr, int32_t color)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
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
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
        return result;
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeStallProtect(uint8_t id, uint8_t addr, bool en)
{
    int8_t result = writeMotorRawParameter(id, addr, I2C_STALL_PROTECTION_REG, 1, 1, en);
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
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
    if (result < 0) {
        return result;
    }
    if (bps != readBPS(id, addr)) {
        return ROLLER_WRITE_FAILED;
    }
    return ROLLER_WRITE_SUCCESS;
}

bool UnitRollerCAN::sendData(uint8_t can_id, uint8_t cmd_id, uint16_t option, const uint8_t *data)
{
    ping_message.flags |= (1 << 0);
    ping_message.identifier = 0x00000000 | (cmd_id << 24) | (option << 16) | can_id;
#if defined UNIT_ROLLER_DEBUG
    Serial.printf("send data: 0x%02X ", ping_message.identifier);
    for (int i = 0; i < 8; i++) {
        printf("0x%02X ", ping_message.data[i]);
    }
    Serial.printf("\n");
#else
#endif
    for (int i = 0; i < 8; i++) {
        ping_message.data[i] = data[i];
    }
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

int8_t UnitRollerCAN::setMotorParameter(uint8_t id, uint16_t parameterId, int32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::setMotorPID(uint8_t id, uint16_t parameterId, uint32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::getMotorParameter(uint8_t id, uint16_t parameterId)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    if (!sendData(id, ROLLERCAN_READ_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_SUCCESS;
    }
    int8_t result = receiveData();
    if (result < 0) {
        releaseMutex();
        return result;
    }
    int32_t returnValue = handleValue(readData[4], readData[5], readData[6], readData[7]);
#if defined UNIT_ROLLER_DEBUG
    Serial.printf("Value:%d\n", returnValue);
#else
#endif
    releaseMutex();
    return returnValue;
}

int32_t UnitRollerCAN::readMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId)
{
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    canData[0] = (parameterId & 0xFF);
    canData[1] = (parameterId >> 8) & 0xFF;
    canData[2] = addr;
    if (!sendData(id, ROLLERCAN_READ_I2C_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t result = receiveData();
    if (result < 0) {
        releaseMutex();
        return result;
    }
    int32_t returnValue = handleValue(readData[4], readData[5], readData[6], readData[7]);
#if defined UNIT_ROLLER_DEBUG
    Serial.printf("returnValue:%d\n", returnValue);
#else
#endif
    releaseMutex();
    return returnValue;
}

int8_t UnitRollerCAN::writeMotorParameter(uint8_t id, uint8_t addr, uint16_t parameterId, int32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}
int8_t UnitRollerCAN::writeMotorPID(uint8_t id, uint8_t addr, uint16_t parameterId, uint32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int32_t UnitRollerCAN::readMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg)
{
    memset(canData, 0, sizeof(canData));
    int8_t result = writeMotorRawParameter(id, addr, reg, 1, 0, 0);
    if (result < 0) {
        return result;
    }
    acquireMutex();
    memset(canData, 0, sizeof(canData));
    canData[0] = addr;
    canData[1] = 4;
    if (!sendData(id, ROLLERCAN_READ_I2C_RAW_CMD, 0, canData)) {
        releaseMutex();
        return ROLLER_WRITE_FAILED;
    }
    int8_t res = receiveData();
    if (res < 0) {
        releaseMutex();
        return res;
    }
    int32_t returnValue = handleValue(readData[0], readData[1], readData[2], readData[3]);
    releaseMutex();
    return returnValue;
}

int8_t UnitRollerCAN::writeMotorRawParameter(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len,
                                             int32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRollerCAN::writeMotorRawPID(uint8_t id, uint8_t addr, uint8_t reg, uint8_t option, uint8_t len,
                                       uint32_t parameterData)
{
    acquireMutex();
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
    receiveData();
    releaseMutex();
    return ROLLER_WRITE_SUCCESS;
}
