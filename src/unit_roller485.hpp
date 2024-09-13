/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __UNIT_ROLLER485_H
#define __UNIT_ROLLER485_H

#include "Arduino.h"

#define UNIT_ROLLER_DEBUG Serial  // This macro definition can be annotated without sending and receiving data prints
//      Define the serial port you want to use, e.g., Serial1 or Serial2
#if defined UNIT_ROLLER_DEBUG
#define serialPrint(...)   UNIT_ROLLER_DEBUG.print(__VA_ARGS__)
#define serialPrintln(...) UNIT_ROLLER_DEBUG.println(__VA_ARGS__)
#define serialPrintf(...)  UNIT_ROLLER_DEBUG.printf(__VA_ARGS__)
#define serialFlush()      UNIT_ROLLER_DEBUG.flush()
#else

#endif
// Macro definitions for error codes
/**
 * @brief Indicates a successful write operation.
 */
#define ROLLER485_WRITE_SUCCESS (1)

/**
 * @brief Indicates a failure during write or read operations.
 */
#define ROLLER485_WRITE_FAILED (0)

/**
 * @brief Indicates a failure in CRC check.
 */
#define ROLLER485_CRC_CHECK_FAIL (-1)

/**
 * @brief Indicates a timeout in serial communication.
 */
#define ROLLER485_SERIAL_TIMEOUT (-2)

/**
 * @brief Indicates an unexpected response received from the device.
 */
#define ROLLER485_UNEXPECTED_RESPONSE (-3)

/**
 * @brief Indicates a failure in sending serial data.
 */
#define ROLLER485_SERIAL_SEND_FAILURE (-4)

/**
 * @brief Command to set output parameters of the Unit Roller.
 */
#define ROLLER485_SET_OUTPUT_CMD (0x00)

/**
 * @brief Command to set the mode of the Unit Roller.
 */
#define ROLLER485_SET_MODE_CMD (0x01)

/**
 * @brief Command to remove protection features from the Unit Roller.
 */
#define ROLLER485_REMOVE_PROTECTION_CMD (0x06)

/**
 * @brief Command to save settings or data to flash memory.
 */
#define ROLLER485_SAVE_FLASH_CMD (0x07)

/**
 * @brief Command to interact with the encoder.
 */
#define ROLLER485_ENCODER_CMD (0x08)

/**
 * @brief Command to handle button interactions.
 */
#define ROLLER485_BUTTON_CMD (0x09)

/**
 * @brief Command to set RGB parameters.
 */
#define ROLLER485_SET_RGB_CMD (0x0A)

/**
 * @brief Command to set the baud rate for communication.
 */
#define ROLLER485_BAUD_RATE_CMD (0x0B)

/**
 * @brief Command to set the motor ID.
 */
#define ROLLER485_SET_MOTOR_ID_CMD (0x0C)

/**
 * @brief Command to enable jam protection features.
 */
#define ROLLER485_JAM_PROTECTION_CMD (0x0D)

/**
 * @brief command is used to set the speed in speed mode.
 */
#define ROLLER485_SPEED_MODE_CMD (0x20)

/**
 * @brief command is used to set the speed PID in speed mode.
 */
#define ROLLER485_SPEED_PID_CMD (0x21)

/**
 * @brief command is used to set the opsition in opsition mode.
 */
#define ROLLER485_POSITION_MODE_CMD (0x22)

/**
 * @brief command is used to set the opsition PID in speed mode.
 */
#define ROLLER485_POSITION_PID_CMD (0x23)

/**
 * @brief command is used to set the current in current mode.
 */
#define ROLLER485_CURRENT_MODE_CMD (0x24)

/**
 * @brief Command to read back parameter group 0.
 */
#define ROLLER485_READ_BACK0_CMD (0x40)

/**
 * @brief Command to read back parameter group 1.
 */
#define ROLLER485_READ_BACK1_CMD (0x41)

/**
 * @brief Command to read back parameter group 2.
 */
#define ROLLER485_READ_BACK2_CMD (0x42)

/**
 * @brief Command to read back parameter group 3.
 */
#define ROLLER485_READ_BACK3_CMD (0x43)

/**
 * @brief Command to read I2C data from address 1.
 */
#define ROLLER485_READ_I2C_DATA1_CMD (0x60)

/**
 * @brief Command to read I2C data from address 2.
 */
#define ROLLER485_READ_I2C_DATA2_CMD (0x62)

/**
 * @brief Command to write I2C data to address 1.
 */
#define ROLLER485_WRITE_I2C_DATA1_CMD (0x61)

/**
 * @brief Command to write I2C data to address 1.
 */
#define ROLLER485_WRITE_I2C_DATA2_CMD (0x63)

/**
 * @brief Length of I2C data being transferred.
 *
 * This constant defines the standard length for I2C data transactions.
 */
#define ROLLER485_I2C_DATA_LEN (0x10)

/**
 * @brief  motor configuration Settings or read register address..
 */
#define ROLLER485_I2C_MOTORCONFIG_REG (0x00)

/**
 * @brief Motor id, 485 bps, RGB brightness Settings or read register address.
 */
#define ROLLER485_I2C_DISPOSTION_REG (0x10)

/**
 * @brief position mode sets the maximum current or reads the register address..
 */
#define ROLLER485_I2C_POSTIONMODECURRENT_REG (0x20)

/**
 * @brief Temperature, voltage read, RGB, encoder Settings or read register addresses.
 */
#define ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG (0X30)

/**
 * @brief Speed mode sets the speed  or reads the register address.
 */
#define ROLLER485_I2C_SPEEDMODE_REG (0x40)

/**
 * @brief Speed mode sets the maximum current or reads the register address.
 */
#define ROLLER485_I2C_SPEEDMODECURRENT_REG (0x50)

/**
 * @brief Configure or read PID to set the register address in speed mode
 */
#define ROLLER485_I2C_SPEEDMODEPID_REG (0x70)

/**
 * @brief Position mode sets the position  or reads the register address.
 */
#define ROLLER485_I2C_POSITIONMODE_REG (0x80)

/**
 * @brief Configure or read PID to set the register address in position mode
 */
#define ROLLER485_I2C_POSITIONMODEPID_REG (0xA0)

/**
 * @brief Current mode sets the current  or reads the register address.
 */
#define ROLLER485_I2C_CURRENTMODE_REG (0xB0)

/**
 * @brief Register for storing the device ID in flash memory.
 */
#define ROLLER485_I2C_ID_FLASH_REG (0xF0)

/**
 * @brief Register that contains the firmware version information.
 */
#define ROLLER485_I2C_VERSION_REG (0xFE)

/**
 * @brief Read or set the register address of the device I2C address.
 */
#define ROLLER485_I2C_READ_I2CADDRESS (0xFF)

class UnitRoller485 {
private:
    bool mutexLocked;
    static const size_t BUFFER_SIZE = 128;
    char buffer[BUFFER_SIZE];
    uint8_t motorData[15] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint8_t Readback[4] = {0x00, 0x00, 0x00, 0x00};
    // 485->i2c
    uint8_t readI2cNum1[5]  = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t readI2cNum2[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t writeI2cNum[25] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int8_t verifyResponse(const char *responseBuffer, size_t responseSize, uint8_t expectedResponse,
                          bool verifyResponse);
    int8_t verifyData(uint8_t *data, size_t length, bool verify);
    int32_t handleValue(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3);

public:
    HardwareSerial *serialPort;

    void begin(HardwareSerial *serial, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1,
               int8_t txPin = -1, bool invert = false, unsigned long timeout_ms = 10000UL,
               uint8_t rxfifo_full_thrhd = 112UL);
    void acquireMutex();
    void releaseMutex();
    bool sendData(const char *data, size_t length);
    size_t readData();
    uint8_t crc8(uint8_t *data, uint8_t len);

    int8_t setOutput(uint8_t id, bool motorEn);
    int8_t setMode(uint8_t id, uint8_t mode);
    int8_t setRemoveProtection(uint8_t id, bool protectionEn);
    int8_t setSaveFlash(uint8_t id, bool saveFlashEn);
    int8_t setEncoder(uint8_t id, int32_t encoder);
    int8_t setButton(uint8_t id, bool buttonEn);
    int8_t setRGB(uint8_t id, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, uint8_t rgbBrightness, uint8_t rgbMode);
    int8_t setBaudRate(uint8_t id, uint8_t baudRate);
    int8_t setMotorId(uint8_t id, uint8_t motorId);
    int8_t setJamProtection(uint8_t id, bool protectionEn);
    int32_t setSpeedMode(uint8_t id, int32_t speed, int32_t current);
    int32_t setSpeedPID(uint8_t id, uint32_t speedP, uint32_t speedI, uint32_t speedD);
    int32_t setPositionMode(uint8_t id, int32_t position, int32_t current);
    int32_t setPositionPID(uint8_t id, uint32_t positionP, uint32_t positionI, uint32_t positionD);
    int32_t setCurrentMode(uint8_t id, int32_t current);

    int32_t getActualSpeed(uint8_t id);
    int32_t getActualPosition(uint8_t id);
    int32_t getActualCurrent(uint8_t id);
    int32_t getEncoder(uint8_t id);
    int32_t getActualVin(uint8_t id);
    int32_t getActualTemp(uint8_t id);
    int32_t getMotorId(uint8_t id);
    int8_t getSpeedPID(uint8_t id, double *speedPID);
    int8_t getPositionPID(uint8_t id, double *positionPID);
    int8_t getRGB(uint8_t id, uint8_t *rgbValues);
    int8_t getMode(uint8_t id);
    int8_t getStatus(uint8_t id);
    int8_t getError(uint8_t id);
    int8_t getRGBMode(uint8_t id);
    int8_t getRGBBrightness(uint8_t id);
    int8_t getBaudRate(uint8_t id);
    int8_t getButton(uint8_t id);
    // 485->i2c
    int8_t readI2c(uint8_t id, uint8_t address, uint8_t dataLen, uint8_t *readBuffer);
    int8_t readI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1, uint8_t dataLen,
                   uint8_t *readBuffer);
    int8_t readSpeedPID(uint8_t id, uint8_t address, double *speedPID);
    int8_t readPositionPID(uint8_t id, uint8_t address, double *positionPID);
    int8_t readRGB(uint8_t id, uint8_t address, uint8_t *rgbValues);

    int8_t readOutput(uint8_t id, uint8_t address);
    int8_t readMode(uint8_t id, uint8_t address);
    int8_t readStatus(uint8_t id, uint8_t address);
    int8_t readError(uint8_t id, uint8_t address);
    int8_t readButton(uint8_t id, uint8_t address);
    int8_t readStallProtection(uint8_t id, uint8_t address);
    int8_t readBaudRate(uint8_t id, uint8_t address);
    int8_t readRgbBrightness(uint8_t id, uint8_t address);
    int8_t readVersion(uint8_t id, uint8_t address);
    int8_t readI2cAddress(uint8_t id, uint8_t address);
    int8_t readMotorId(uint8_t id, uint8_t address);

    int32_t readSpeed(uint8_t id, uint8_t address);
    int32_t readSpeedCurrent(uint8_t id, uint8_t address);
    int32_t readPosition(uint8_t id, uint8_t address);
    int32_t readPositionCurrent(uint8_t id, uint8_t address);
    int32_t readCurrent(uint8_t id, uint8_t address);
    int32_t readEncoder(uint8_t id, uint8_t address);
    int32_t readVin(uint8_t id, uint8_t address);
    int32_t readTemp(uint8_t id, uint8_t address);

    int8_t writeI2c(uint8_t id, uint8_t address, uint8_t *data, uint8_t dataLen, bool stopBit);
    int8_t writeI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1, uint8_t *data,
                    uint8_t dataLen);
    int8_t writeMotorConfig(uint8_t id, uint8_t address, bool motorEn, uint8_t mode, bool removeProtection,
                            bool buttonEn, bool stallProtection);
    int8_t writeDisposition(uint8_t id, uint8_t address, uint8_t motorId, uint8_t baudRate, uint8_t rgbBrightness);
    int8_t writeSpeedMode(uint8_t id, uint8_t address, int32_t speed);
    int8_t writeSpeedModeCurrent(uint8_t id, uint8_t address, int32_t current);
    int8_t writeSpeedModePID(uint8_t id, uint8_t address, uint32_t speedP, uint32_t speedI, uint32_t speedD);
    int8_t writePositionMode(uint8_t id, uint8_t address, int32_t position);
    int8_t writePositionModeCurrent(uint8_t id, uint8_t address, int32_t current);
    int8_t writePositionModePID(uint8_t id, uint8_t address, uint32_t positionP, uint32_t positionI,
                                uint32_t positionD);
    int8_t writeCurrentMode(uint8_t id, uint8_t address, int32_t current);
    int8_t writeSetRGB(uint8_t id, uint8_t address, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, uint8_t rgbMode);
    int8_t writeEncoderMode(uint8_t id, uint8_t address, int32_t encoder);
    int8_t writeI2cId(uint8_t id, uint8_t address, bool saveFlashEn, uint8_t newAddress);
};
#endif
