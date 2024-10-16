/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#ifndef __UNIT_ROLLER485_H
#define __UNIT_ROLLER485_H

#include "Arduino.h"
#include "unit_roller_common.hpp"

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

class UnitRoller485 {
public:
    HardwareSerial *serialPort;

    /**
     * @brief Initializes the hardware serial communication interface.
     *
     * This function sets up the serial communication for the UnitRoller485 class,
     * configuring parameters such as baud rate, pin assignments, signal inversion,
     * and timeouts. It is essential to call this function before attempting to =
     * communicate using the serial interface.
     *
     * @param serial Pointer to a HardwareSerial object used for the serial communication.
     * @param baud The baud rate for serial communication (e.g., 9600, 115200).
     * @param config Configuration parameter for serial settings (default: SERIAL_8N1).
     *               This specifies data bits, parity, and stop bits.
     * @param rxPin The GPIO pin number designated for receiving data (default: -1 for auto).
     * @param txPin The GPIO pin number designated for transmitting data (default: -1 for auto).
     * @param dirPin The GPIO pin number used for direction control in RS-485 mode
     *               (default: -1 indicates no direction control).
     * @param invert A boolean value indicating whether to invert the signal (true for RS-485
     *               communication, which may require signal inversion).
     * @param timeout_ms The read timeout duration in milliseconds; specifies how long
     *                   to wait for incoming data before timing out (default: 10000 ms).
     * @param rxfifo_full_thrhd The threshold for the received FIFO buffer full condition;
     *                          when the buffer reaches this size, an overflow warning may be triggered (default: 112).
     */
    void begin(HardwareSerial *serial, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1,
               int8_t txPin = -1, int8_t dirPin = -1, bool invert = false, unsigned long timeout_ms = 10000UL,
               uint8_t rxfifo_full_thrhd = 112UL);

    /**
     * @brief sends data
     *
     * Send data through a serial port in the UnitRoller485 class.
     *
     * @param data points to a character pointer to send data
     * @param length Length of the data to be sent
     *
     * @return true if the data is sent statefully. Otherwise return false
     */
    bool sendData(const char *data, size_t length);

    /**
     * @brief reads data
     *
     * Reads data from a serial port in class UnitRoller485 and applies a timeout during the read.
     * Read data is first stored in an internal buffer, and then specific header information (such as "AA 55") is
     * removed.
     *
     * @return Number of bytes read statefully (excluding removed headers)
     */
    size_t readData();

    /**
     * @brief calculates 8-bit CRC
     *
     * Calculates and returns an 8-bit CRC value based on the given data array and length.
     *
     * @param data Data array pointer that contains the data to calculate CRC
     * @param len Length of the data array
     *
     * @return Calculated 8-bit CRC value
     */
    uint8_t crc8(uint8_t *data, uint8_t len);

    /**
     * @brief Set motor enable or off
     * @param id  Motor equipment id   Value range:0~255
     * @param motorEn     motor enable or off
     * @return The result or error code of the response validation
     */
    int8_t setOutput(uint8_t id, bool motorEn);

    /**
     * @brief Set motor mode
     * @param id  Motor equipment id   Value range:0~255
     * @param mode    Mode setting   1: Speed Mode 2: Position Mode 3: Current
     * Mode 4. Encoder Mode
     * @return The result or error code of the response validation
     */
    int8_t setMode(uint8_t id, roller_mode_t mode);

    /**
     * @brief Set Remove protection
     * @param id  Motor equipment id   Value range:0~255
     * @param protectionEn  Release Jam protection: Send 1 to unprotect
     * @return The result or error code of the response validation
     */
    int8_t setRemoveProtection(uint8_t id, bool protectionEn);

    /**
     * @brief Save to flash
     * @param id  Motor equipment id   Value range:0~255
     * @param saveFlashEn Save to flash: Send 1 save parameters to flash
     * @return The result or error code of the response validation
     */
    int8_t setSaveFlash(uint8_t id, bool saveFlashEn);

    /**
     * @brief Set encoder value
     * @param id  Motor equipment id   Value range:0~255
     * @param encoder  Encoder value(int32_t)
     * @return The result or error code of the response validation
     */
    int8_t setEncoder(uint8_t id, int32_t encoder);

    /**
     * @brief Set button switching mode enable
     * @param id  Motor equipment id   Value range:0~255
     * @param buttonEn  0: Off; 1: Press and hold for 5S to switch modes in running
     * mode.
     * @return The result or error code of the response validation
     */
    int8_t setButton(uint8_t id, bool buttonEn);

    /**
     * @brief Set RGB (RGB Mode and RGB Brightness can be save to flash)
     * @param id  Motor equipment id   Value range:0~255
     * @param rgbR   Value range:0~255
     * @param rgbG   Value range:0~255
     * @param rgbB   Value range:0~255
     * @param rgbBrightness   Value range:0~100
     * @param rgbMode      Mode： 0, Sys-default 1,user-define    Default
     * value：0
     * @return The result or error code of the response validation
     */
    int8_t setRGB(uint8_t id, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, uint8_t rgbBrightness, roller_rgb_t rgbMode);

    /**
     * @brief Set baud rate (can be save to flash)
     * @param id  Motor equipment id   Value range:0~255
     * @param baudRate  BPS: 0,115200bps; 1, 19200bps; 2, 9600bps; Default
     * value：0
     * @return The result or error code of the response validation
     */
    int8_t setBaudRate(uint8_t id, roller_bps_t baudRate);

    /**
     * @brief Set motor id (can be save to flash)
     * @param id  Motor equipment id   Value range:0~255
     * @param motorId     Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t setMotorId(uint8_t id, uint8_t motorId);

    /**
     * @brief Set motor jam protection
     * @param id  Motor equipment id   Value range:0~255
     * @param protectionEn    Motor Jam Protection: 0,Disable; 1, Enable
     * @return The result or error code of the response validation
     */
    int8_t setJamProtection(uint8_t id, bool protectionEn);

    /**
     * @brief  Set speed mode configur ation
     * @param id  Motor equipment id   Value range:0~255
     * @param speed      Speed  Value:-21000000 ~21000000
     * @param current    current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int32_t setSpeedMode(uint8_t id, int32_t speed, int32_t current);

    /**
     * @brief  Set the speed mode PID configuration
     * @param id  Motor equipment id   Value range:0~255
     * @param speedP    For example: mottor P=0.004, speedP setting
     * value=0.004*100000=400,
     * @param speedI    For example: mottor I=0.002, speedI setting
     * value=0.002*10000000=20000,
     * @param speedD    For example: mottor D=16.00, speedD setting
     * value=16.00*100000=1600000,
     * @return The result or error code of the response validation
     */
    int32_t setSpeedPID(uint8_t id, uint32_t speedP, uint32_t speedI, uint32_t speedD);

    /**
     * @brief  Set position mode configur ation
     * @param id  Motor equipment id   Value range:0~255
     * @param position     Speed  Value:-21000000 ~21000000
     * @param current    current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int32_t setPositionMode(uint8_t id, int32_t position, int32_t current);

    /**
     * @brief  Set the position mode PID configuration
     * @param id  Motor equipment id   Value range:0~255
     * @param positionP    For example: mottor P=0.004, positionP setting
     * value=0.004*100000=400,
     * @param positionI    For example: mottor I=0.002, positionI setting
     * value=0.002*10000000=20000,
     * @param positionD    For example: mottor D=16.00, positionD setting
     * value=16.00*100000=1600000,
     * @return The result or error code of the response validation
     */
    int32_t setPositionPID(uint8_t id, uint32_t positionP, uint32_t positionI, uint32_t positionD);

    /**
     * @brief  Set current mode configur ation
     * @param id  Motor equipment id   Value range:0~255
     * @param current    current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int32_t setCurrentMode(uint8_t id, int32_t current);

    /**
     * @brief  Get motor speed
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getActualSpeed(uint8_t id);

    /**
     * @brief   Get motor position
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getActualPosition(uint8_t id);

    /**
     * @brief  Get motor current
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getActualCurrent(uint8_t id);

    /**
     * @brief  Get motor encoder
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getEncoder(uint8_t id);
    /**
     * @brief  Get motor vin
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getActualVin(uint8_t id);

    /**
     * @brief  Get motor temperature
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getActualTemp(uint8_t id);

    /**
     * @brief  Get motor id
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int32_t getMotorId(uint8_t id);

    /**
     * @brief  Get  motor speed PID
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getSpeedPID(uint8_t id, double *speedPID);

    /**
     * @brief  Get  motor position PID
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getPositionPID(uint8_t id, double *positionPID);

    /**
     * @brief  Get motor RGB  mode
     * @param id  Motor equipment id   Value range:0~255
     */
    int8_t getRGB(uint8_t id, uint8_t *rgbValues);

    /**
     * @brief  Get motor mode
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getMode(uint8_t id);

    /**
     * @brief  Get motor status
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getStatus(uint8_t id);

    /**
     * @brief  Get motor error
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getError(uint8_t id);

    /**
     * @brief  Get motor RGB  mode
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getRGBMode(uint8_t id);

    /**
     * @brief  Get motor RGB  rgbBrightness
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getRGBBrightness(uint8_t id);

    /**
     * @brief  Get motor baudrate
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getBaudRate(uint8_t id);

    /**
     * @brief  Get motor button
     * @param id  Motor equipment id   Value range:0~255
     * @return The result or error code of the response validation
     */
    int8_t getButton(uint8_t id);
    // 485->i2c

    /**
     * @brief reads I2C device data
     *
     * Reads data from the I2C device with the specified ID, address, and data length, and stores the read data into the
     * given read buffer.
     *
     * @param id ID of the I2C device
     * @param address Indicates the address of the I2C device
     * @param dataLen Length of data to be read
     * @param readBuffer Buffer used to store the read data
     *
     * @return The number of bytes read statefully, and an error code if an error occurs
     */
    int8_t readI2c(uint8_t id, uint8_t address, uint8_t dataLen, uint8_t *readBuffer);

    /**
     * @brief reads data from the I2C device
     *
     * Reads data from the I2C device with the specified ID, address, address length, register address, and data length,
     * and stores the read data into the given read buffer.
     *
     * @param id ID of the I2C device
     * @param address Base address of the I2C device
     * @param addressLen The length of the address (usually 1 or 2, depending on the register address structure of the
     * I2C device)
     * @param regByte0 High byte of register address (if addressLen is 2, otherwise ignored)
     * @param regByte1 Low byte of register address
     * @param dataLen Length of data to be read
     * @param readBuffer Buffer used to store the read data
     *
     * @return The number of bytes read statefully, and an error code if an error occurs
     */
    int8_t readI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1, uint8_t dataLen,
                   uint8_t *readBuffer);

    /**
     * @brief  reads the motor speed PID from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @param speedPID Pointer to type double to store the read speedPID parameter
     *
     * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
     * returned
     */
    int8_t readSpeedPID(uint8_t id, uint8_t address, double *speedPID);

    /**
     * @brief  reads the motor postion PID from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @param positionPID Pointer to type double to store the read postionPID parameter
     *
     * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
     * returned
     */
    int8_t readPositionPID(uint8_t id, uint8_t address, double *positionPID);

    /**
     * @brief  reads the motor rgb from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @param rgbValues Pointer to type double to store the read rgbValues parameter
     *
     * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
     * returned
     */
    int8_t readRGB(uint8_t id, uint8_t address, uint8_t *rgbValues);

    /**
     * @brief reads the output from the I2C device
     *
     * Read the output value from the I2C device with the specified ID and address (possibly a motor configuration
     * register).
     *
     * @param id   Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     *
     * @return Read the output value and return an error code if an error occurs
     */
    int8_t readOutput(uint8_t id, uint8_t address);

    /**
     * @brief  reads the mode from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the mode value and return an error code if an error occurs
     */
    int8_t readMode(uint8_t id, uint8_t address);

    /**
     * @brief  reads the status from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the status value and return an error code if an error occurs
     */
    int8_t readStatus(uint8_t id, uint8_t address);

    /**
     * @brief  reads the error from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device  Value range:0~127
     * @return Read the error value and return an error code if an error occurs
     */
    int8_t readError(uint8_t id, uint8_t address);

    /**
     * @brief   reads the button from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the button value and return an error code if an error occurs
     */
    int8_t readButton(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor position over range protection from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the motor position over range protection value and return an error code if an error occurs
     */
    int8_t readRangeProtection(uint8_t id, uint8_t address);

    /**
     * @brief  reads the stall protection from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the stall protection value and return an error code if an error occurs
     */
    int8_t readStallProtection(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor baud rate from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the baud rate value and return an error code if an error occurs
     */
    int8_t readBaudRate(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor rgbbrightness from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the rgbbrightness value and return an error code if an error occurs
     */
    int8_t readRgbBrightness(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor version from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the version value and return an error code if an error occurs
     */
    int8_t readVersion(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor i2caAddress from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the i2caAddress value and return an error code if an error occurs
     */
    int8_t readI2cAddress(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor id from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the id  value and return an error code if an error occurs
     */
    int8_t readMotorId(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor speed from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the speed  value and return an error code if an error occurs
     */
    int32_t readSpeed(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor speed current from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the speed current value and return an error code if an error occurs
     */
    int32_t readSpeedCurrent(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor speed from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the actual speed readback  value and return an error code if an error occurs
     */
    int32_t readSpeedReadback(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor position from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the postion value and return an error code if an error occurs
     */
    int32_t readPosition(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor position current from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the position current  value and return an error code if an error occurs
     */
    int32_t readPositionCurrent(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor position from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the actual postion readback  value and return an error code if an error occurs
     */
    int32_t readPositionReadback(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor current from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the current value and return an error code if an error occurs
     */
    int32_t readCurrent(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor current from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the actual current readback  value and return an error code if an error occurs
     */
    int32_t readCurrentReadback(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor encoder from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the encoder value and return an error code if an error occurs
     */
    int32_t readEncoder(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor vin from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the vin value and return an error code if an error occurs
     */
    int32_t readVin(uint8_t id, uint8_t address);

    /**
     * @brief  reads the motor temp from the I2C device
     * @param id  Motor equipment id   Value range:0~255
     * @param address Address of the I2C device Value range:0~127
     * @return Read the temp value and return an error code if an error occurs
     */
    int32_t readTemp(uint8_t id, uint8_t address);

    /**
     * @brief  wirte i2c
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address
     * @param data   Write data
     * @param dataLen  Write data len
     * @param stopBit  Stop bit  Usually false
     */
    int8_t writeI2c(uint8_t id, uint8_t address, uint8_t *data, uint8_t dataLen, bool stopBit);

    /**
     * @brief  wirte i2c
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address
     * @param addressLen Device address  i2c reg address len: 0, 1 byte address; 1, 2 bytes address
     * i2c reg address:
     * 1. i2c reg address len = 0, i2c reg address = i2c reg address bytes0
     * 2. i2c reg address len = 1, i2creg address = i2c reg address bytes0 + (i2c reg address bytes1 * 255)
     * @param regByte0  Register byte
     * @param regByte1  Register byte
     * @param data   Write data
     * @param dataLen  Write data len
     */
    int8_t writeI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1, uint8_t *data,
                    uint8_t dataLen);

    /**
     * @brief  write MotorConfig
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address
     * @param motorEn    motor enable or off
     * @param mode    Mode setting   1: Speed Mode 2: Position Mode 3: Current
     * Mode 4. Encoder Mode
     * @param rangeProtection   Motor Position Over Range Protection: 0,Disable; 1, Enable
     * @param removeProtection  Release Jam protection: Send 1 to unprotect
     * @param buttonEn  0: Off; 1: Press and hold for 5S to switch modes in running
     * mode.
     * @param stallProtection   Motor Stall Protection:: 0,Disable; 1, Enable
     * @return The result or error code of the response validation
     */
    int8_t writeMotorConfig(uint8_t id, uint8_t address, bool motorEn, roller_mode_t mode, bool rangeProtection,
                            bool removeProtection, bool buttonEn, bool stallProtection);

    /**
     * @brief  write_disposition
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param motorId     Value range:0~255
     * @param baudRate    BPS: 0,115200bps; 1, 19200bps; 2, 9600bps;Default value：0
     * @param rgbBrightness   Value range:0~100
     * @return The result or error code of the response validation
     */
    int8_t writeDisposition(uint8_t id, uint8_t address, uint8_t motorId, roller_bps_t baudRate, uint8_t rgbBrightness);

    /**
     * @brief  write_speedMode
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address  Value range:0~127
     * @param speed    Speed  Value:-21000000 ~21000000
     * @return The result or error code of the response validation
     */
    int8_t writeSpeedMode(uint8_t id, uint8_t address, int32_t speed);

    /**
     * @brief  write speedMode  current
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param current    current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int8_t writeSpeedModeCurrent(uint8_t id, uint8_t address, int32_t current);

    /**
     * @brief  write speedMode PID
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param speedP    For example: mottor P=0.004, speedP setting
     * value=0.004*100000=400,
     * @param speedI    For example: mottor I=0.002, speedI setting
     * value=0.002*10000000=20000,
     * @param speedD    For example: mottor D=16.00, speedD setting
     * value=16.00*100000=1600000,
     * @return The result or error code of the response validation
     */
    int8_t writeSpeedModePID(uint8_t id, uint8_t address, uint32_t speedP, uint32_t speedI, uint32_t speedD);

    /**
     * @brief  write positionMode
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param position   position  Value:-21000000 ~21000000
     * @return The result or error code of the response validation
     */
    int8_t writePositionMode(uint8_t id, uint8_t address, int32_t position);

    /**
     * @brief  write positionMode current
     * @param address Device address   Value range:0~127
     * @param id  Motor equipment id   Value range:0~255
     * @param current    current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int8_t writePositionModeCurrent(uint8_t id, uint8_t address, int32_t current);

    /**
     * @brief  write positionMode PID
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param positionP    For example: mottor P=0.004, position_P setting
     * value=0.004*100000=400,
     * @param positionI    For example: mottor I=0.002, position_I setting
     * value=0.002*10000000=20000,
     * @param positionD    For example: mottor D=16.00, position_D setting
     * value=16.00*100000=1600000,
     * @return The result or error code of the response validation
     */
    int8_t writePositionModePID(uint8_t id, uint8_t address, uint32_t positionP, uint32_t positionI,
                                uint32_t positionD);

    /**
     * @brief  write currentMode
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param current   current  Value:-1200 ~1200
     * @return The result or error code of the response validation
     */
    int8_t writeCurrentMode(uint8_t id, uint8_t address, int32_t current);

    /**
     * @brief  write setRGB
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param rgbR   Value range:0~255
     * @param rgbG   Value range:0~255
     * @param rgbB   Value range:0~255
     * @param rgbModeRGB      Mode： 0, Sys-default 1, User-define    Default
     * value：0
     * @return The result or error code of the response validation
     */
    int8_t writeSetRGB(uint8_t id, uint8_t address, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, roller_rgb_t rgbMode);

    /**
     * @brief  write encoder
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:0~127
     * @param encoder    encoder  uint32_t
     * @return The result or error code of the response validation
     */
    int8_t writeEncoderMode(uint8_t id, uint8_t address, int32_t encoder);

    /**
     * @brief  write  i2c id
     * @param id  Motor equipment id   Value range:0~255
     * @param address Device address   Value range:1~127
     * @param saveFlashEn Save to flash: Send 1 save parameters to flash
     * @param newAddress  new address  Value range:1~127
     * @return The result or error code of the response validation
     */
    int8_t writeI2cId(uint8_t id, uint8_t address, bool saveFlashEn, uint8_t newAddress);

private:
    bool mutexLocked;
    int8_t dirPin;
    static const size_t BUFFER_SIZE = 128;
    char buffer[BUFFER_SIZE];
    uint8_t motorData[15] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t Readback[4]   = {0x00, 0x00, 0x00, 0x00};
    // 485->i2c
    uint8_t readI2cNum1[5]  = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t readI2cNum2[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t writeI2cNum[25] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    /**
    * @brief verifies UART response data
    *
    Verify received UART response data and check that its CRC checksum matches expectations.
    * If needed, the first byte of the response and (if validation is enabled) protocol-specific bytes are also checked.
    *
    * @param responseBuffer Buffer of response data
    * @param responseSize Size of response data (in bytes)
    * @param expectedResponse the first byte of the expected response
    * @param verifyResponse Whether additional response validation is enabled (e.g. I2C write/read operations)
    *
    * @return returns WRITE_stateif the validation succeeds, otherwise returns the corresponding error code
    */
    int8_t verifyResponse(const char *responseBuffer, size_t responseSize, uint8_t expectedResponse,
                          bool verifyResponse);

    /**
     * @brief verifies that data is sent and received concurrently
     *
     * This function is responsible for sending the given data and waiting for the received response. If validation is
     * enabled, the response data is also validated.
     *
     * @param data Pointer to the data to be sent
     * @param length Length of the data
     * @param verify Whether response verification is enabled
     *
     * @return The result or error code of the response validation
     * -SERIAL_SEND_FAILURE: data fails to be sent
     * -SERIAL_TIMEOUT: A timeout occurs while waiting for a response
     * - Other values: The result of verifying the response (if the verification was stateful)
     */
    int8_t verifyData(uint8_t *data, size_t length, bool verify);
};

#endif
