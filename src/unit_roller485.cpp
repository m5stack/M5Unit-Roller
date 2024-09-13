/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_roller485.hpp"

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
int8_t UnitRoller485::verifyResponse(const char *responseBuffer, size_t responseSize, uint8_t expectedResponse,
                                     bool verifyResponse)
{
    // Perform CRC check if needed
    uint8_t received_crc   = responseBuffer[responseSize - 1];
    uint8_t calculated_crc = crc8((uint8_t *)responseBuffer, responseSize - 1);

    if (received_crc != calculated_crc) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("CRC check failed. Data may be corrupted.\n");
#else
#endif
        return ROLLER485_CRC_CHECK_FAIL;  // CRC check failed
    }
    // Check if response matches expected
    if (responseBuffer[0] != expectedResponse) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Unexpected response received.\n");
#else
#endif
        return ROLLER485_UNEXPECTED_RESPONSE;  // Unexpected response
    }
    if (verifyResponse) {
        if (responseBuffer[2] != 1) {
#if defined UNIT_ROLLER_DEBUG
            serialPrint("I2C write or read failed\n");
#else
#endif
            return ROLLER485_WRITE_FAILED;
        }
    }
    return ROLLER485_WRITE_SUCCESS;
}
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
int8_t UnitRoller485::verifyData(uint8_t *data, size_t length, bool verify)
{
    // Send data
    if (!sendData(reinterpret_cast<const char *>(data), length)) {
        releaseMutex();
        return ROLLER485_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER485_SERIAL_TIMEOUT;
    }
    int response = verifyResponse(buffer, bytesRead, data[0] + 0x10, verify);
    releaseMutex();
    return response;
}
/**
 * @brief processes values
 *
 * Combines four bytes into a 32-bit integer and returns that value.
 * This function is usually used to process raw data from encoders or other devices.
 *
 * @param byte0 Minimum valid byte
 * @param byte1 Second byte
 * @param byte2 Third byte
 * @param byte3 Highest valid byte
 *
 * @return The combined 32-bit integer
 */
int32_t UnitRoller485::handleValue(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    int32_t newNumerical = 0;
    // Combine bytes to form the encoder value
    newNumerical |= ((int32_t)byte0);  // LSB
    newNumerical |= ((int32_t)byte1 << 8);
    newNumerical |= ((int32_t)byte2 << 16);
    newNumerical |= ((int32_t)byte3 << 24);  // MSB
    return newNumerical;
}

/**
* @brief initializes serial communication
*
Initialize the hardware serial communication interface in the UnitRoller485 class.
*
* @param serial A pointer to a HardwareSerial object for serial communication
* @param baud Baud rate: Specifies the rate of serial communication
* @param config configuration parameter
* @param rxPin Receiving pin number
* @param txPin Send pin number
* @param invert Whether to reverse the signal (for example, for RS-485 communication)
* @param timeout_ms Read timeout (in milliseconds)
* @param rxfifo_full_thrhd Received FIFO buffer full threshold
*/
void UnitRoller485::begin(HardwareSerial *serial, unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin,
                          bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd)
{
    serialPort = serial;
    serialPort->begin(baud, SERIAL_8N1, rxPin, txPin, invert, timeout_ms, rxfifo_full_thrhd);
}
/**
 * @brief Wait for the mutex to be unlocked
 */
void UnitRoller485::acquireMutex()
{
    while (mutexLocked) {
        delay(1);
    }
    mutexLocked = true;
}
/**
 * @brief Release the mutex
 */
void UnitRoller485::releaseMutex()
{
    mutexLocked = false;
}
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
bool UnitRoller485::sendData(const char *data, size_t length)
{
#if defined UNIT_ROLLER_DEBUG
    serialPrint("Sent data: ");
    for (size_t i = 0; i < length; i++) {
        serialPrint("0x");
        serialPrint(data[i], HEX);
        serialPrint(" ");
    }
    serialPrintln();
#else
#endif
    size_t bytesSent = serialPort->write(reinterpret_cast<const uint8_t *>(data), length);
    if (bytesSent != length) {
        return false;
    }
    delay(200);
    return true;
}

/**
 * @brief reads data
 *
 * Reads data from a serial port in class UnitRoller485 and applies a timeout during the read.
 * Read data is first stored in an internal buffer, and then specific header information (such as "AA 55") is removed.
 *
 * @return Number of bytes read statefully (excluding removed headers)
 */
size_t UnitRoller485::readData()
{
    memset(buffer, 0, BUFFER_SIZE);
    size_t bytesRead            = 0;
    unsigned long startTime     = millis();
    unsigned long timeoutMillis = 500;
    // Read data from serial port with timeout
    while (millis() - startTime < timeoutMillis && bytesRead < BUFFER_SIZE) {
        if (serialPort->available() > 0) {
            buffer[bytesRead++] = serialPort->read();
            startTime           = millis();  // Reset timeout start time on stateful read
        }
        delay(1);  // Optional small delay for stability
    }
    // Clean data to remove header "AA 55"
    size_t writeIndex = 0;
    for (size_t i = 0; i < bytesRead; i++) {
        if (buffer[i] == 0xAA && buffer[i + 1] == 0x55) {
            // Skip header
            i++;  // Move to the next byte
        } else {
            // Copy data to a new location
            buffer[writeIndex++] = buffer[i];
        }
    }
    bytesRead = writeIndex;  // Update bytesRead after cleaning
// Print cleaned data
#if defined UNIT_ROLLER_DEBUG
    serialPrint("Received data after cleaning: ");
    for (size_t i = 0; i < bytesRead; i++) {
        serialPrint("0x");
        serialPrint(buffer[i], HEX);
        serialPrint(" ");
    }
    serialPrintln();
#else
#endif
    return bytesRead;
}
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
uint8_t UnitRoller485::crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8c;
            } else
                crc >>= 1;
        }
    }
    return crc;
}
/**
 * @brief Set motor enable or off
 * @param id  Motor equipment id   Value range:0~255
 * @param motorEn     motor enable or off
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setOutput(uint8_t id, bool motorEn)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SET_OUTPUT_CMD;
    motorData[1]                     = id;
    motorData[2]                     = motorEn;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Set motor mode
 * @param id  Motor equipment id   Value range:0~255
 * @param mode    Mode setting   1: Speed Mode 2: Position Mode 3: Current
 * Mode 4. Encoder Mode
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setMode(uint8_t id, uint8_t mode)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SET_MODE_CMD;
    motorData[1]                     = id;
    motorData[2]                     = mode;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Set Remove protection
 * @param id  Motor equipment id   Value range:0~255
 * @param protectionEn  Release Jam protection: Send 1 to unprotect
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setRemoveProtection(uint8_t id, bool protectionEn)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_REMOVE_PROTECTION_CMD;
    motorData[1]                     = id;
    motorData[6]                     = protectionEn;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Save to flash
 * @param id  Motor equipment id   Value range:0~255
 * @param saveFlashEn Save to flash: Send 1 save parameters to flash
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setSaveFlash(uint8_t id, bool saveFlashEn)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SAVE_FLASH_CMD;
    motorData[1]                     = id;
    motorData[2]                     = saveFlashEn;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Set encoder value
 * @param id  Motor equipment id   Value range:0~255
 * @param encoder  Encoder value(int32_t)
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setEncoder(uint8_t id, int32_t encoder)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_ENCODER_CMD;
    motorData[1]                     = id;
    motorData[2]                     = encoder & 0xFF;
    motorData[3]                     = (encoder >> 8) & 0xFF;
    motorData[4]                     = (encoder >> 16) & 0xFF;
    motorData[5]                     = (encoder >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Set button switching mode enable
 * @param id  Motor equipment id   Value range:0~255
 * @param buttonEn  0: Off; 1: Press and hold for 5S to switch modes in running
 * mode.
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setButton(uint8_t id, bool buttonEn)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_BUTTON_CMD;
    motorData[1]                     = id;
    motorData[2]                     = buttonEn;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

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
int8_t UnitRoller485::setRGB(uint8_t id, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, uint8_t rgbBrightness,
                             uint8_t rgbMode)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SET_RGB_CMD;
    motorData[1]                     = id;
    motorData[2]                     = rgbB;
    motorData[3]                     = rgbG;
    motorData[4]                     = rgbR;
    motorData[5]                     = rgbMode;
    motorData[6]                     = rgbBrightness;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief Set baud rate (can be save to flash)
 * @param id  Motor equipment id   Value range:0~255
 * @param baudRate  BPS: 0,115200bps; 1, 19200bps; 2, 9600bps; Default
 * value：0
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setBaudRate(uint8_t id, uint8_t baudRate)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_BAUD_RATE_CMD;
    motorData[1]                     = id;
    motorData[2]                     = baudRate;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    if (baudRate == 0) {
        serialPort->updateBaudRate(115200);
    } else if (baudRate == 1) {
        serialPort->updateBaudRate(19200);
    } else if (baudRate == 2) {
        serialPort->updateBaudRate(9600);
    } else {
        serialPort->updateBaudRate(115200);
    }
    releaseMutex();
    return state;
}

/**
 * @brief Set motor id (can be save to flash)
 * @param id  Motor equipment id   Value range:0~255
 * @param motorId     Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setMotorId(uint8_t id, uint8_t motorId)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SET_MOTOR_ID_CMD;
    motorData[1]                     = id;
    motorData[2]                     = motorId;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}
/**
 * @brief Set motor jam protection
 * @param id  Motor equipment id   Value range:0~255
 * @param protectionEn    Motor Jam Protection: 0,Disable; 1, Enable
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::setJamProtection(uint8_t id, bool protectionEn)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_JAM_PROTECTION_CMD;
    motorData[1]                     = id;
    motorData[2]                     = protectionEn;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}
/**
 * @brief  Set speed mode configur ation
 * @param id  Motor equipment id   Value range:0~255
 * @param speed      Speed  Value:-21000000 ~21000000
 * @param current    current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::setSpeedMode(uint8_t id, int32_t speed, int32_t current)
{
    static const int32_t min        = -21000000;
    static const int32_t max        = 21000000;
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    speed   = constrain(speed, min, max);
    current = constrain(current, currentMin, currentMax);
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SPEED_MODE_CMD;
    motorData[1]                     = id;
    int32_t speedBytes               = (int32_t)(speed * 100);
    int32_t currentBytes             = (int32_t)(current * 100);
    motorData[2]                     = speedBytes & 0xFF;
    motorData[3]                     = (speedBytes >> 8) & 0xFF;
    motorData[4]                     = (speedBytes >> 16) & 0xFF;
    motorData[5]                     = (speedBytes >> 24) & 0xFF;
    motorData[6]                     = currentBytes & 0xFF;
    motorData[7]                     = (currentBytes >> 8) & 0xFF;
    motorData[8]                     = (currentBytes >> 16) & 0xFF;
    motorData[9]                     = (currentBytes >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}
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
int32_t UnitRoller485::setSpeedPID(uint8_t id, uint32_t speedP, uint32_t speedI, uint32_t speedD)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_SPEED_PID_CMD;
    motorData[1]                     = id;
    motorData[2]                     = speedP & 0xFF;
    motorData[3]                     = (speedP >> 8) & 0xFF;
    motorData[4]                     = (speedP >> 16) & 0xFF;
    motorData[5]                     = (speedP >> 24) & 0xFF;
    motorData[6]                     = speedI & 0xFF;
    motorData[7]                     = (speedI >> 8) & 0xFF;
    motorData[8]                     = (speedI >> 16) & 0xFF;
    motorData[9]                     = (speedI >> 24) & 0xFF;
    motorData[10]                    = speedD & 0xFF;
    motorData[11]                    = (speedD >> 8) & 0xFF;
    motorData[12]                    = (speedD >> 16) & 0xFF;
    motorData[13]                    = (speedD >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief  Set position mode configur ation
 * @param id  Motor equipment id   Value range:0~255
 * @param position     Speed  Value:-21000000 ~21000000
 * @param current    current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::setPositionMode(uint8_t id, int32_t position, int32_t current)
{
    static const int32_t min        = -21000000;
    static const int32_t max        = 21000000;
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    position = constrain(position, min, max);
    current  = constrain(current, currentMin, currentMax);
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_POSITION_MODE_CMD;
    motorData[1]                     = id;
    int32_t positionBytes            = (int32_t)(position * 100);
    int32_t currentBytes             = (int32_t)(current * 100);
    motorData[2]                     = positionBytes & 0xFF;
    motorData[3]                     = (positionBytes >> 8) & 0xFF;
    motorData[4]                     = (positionBytes >> 16) & 0xFF;
    motorData[5]                     = (positionBytes >> 24) & 0xFF;
    motorData[6]                     = currentBytes & 0xFF;
    motorData[7]                     = (currentBytes >> 8) & 0xFF;
    motorData[8]                     = (currentBytes >> 16) & 0xFF;
    motorData[9]                     = (currentBytes >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}
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
int32_t UnitRoller485::setPositionPID(uint8_t id, uint32_t positionP, uint32_t positionI, uint32_t positionD)
{
    acquireMutex();
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_POSITION_PID_CMD;
    motorData[1]                     = id;
    motorData[2]                     = positionP & 0xFF;
    motorData[3]                     = (positionP >> 8) & 0xFF;
    motorData[4]                     = (positionP >> 16) & 0xFF;
    motorData[5]                     = (positionP >> 24) & 0xFF;
    motorData[6]                     = positionI & 0xFF;
    motorData[7]                     = (positionI >> 8) & 0xFF;
    motorData[8]                     = (positionI >> 16) & 0xFF;
    motorData[9]                     = (positionI >> 24) & 0xFF;
    motorData[10]                    = positionD & 0xFF;
    motorData[11]                    = (positionD >> 8) & 0xFF;
    motorData[12]                    = (positionD >> 16) & 0xFF;
    motorData[13]                    = (positionD >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief  Set current mode configur ation
 * @param id  Motor equipment id   Value range:0~255
 * @param current    current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::setCurrentMode(uint8_t id, int32_t current)
{
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    current = constrain(current, currentMin, currentMax);
    memset(motorData, 0, sizeof(motorData));
    motorData[0]                     = ROLLER485_CURRENT_MODE_CMD;
    motorData[1]                     = id;
    int32_t currentBytes             = (int32_t)(current * 100);
    motorData[2]                     = currentBytes & 0xFF;
    motorData[3]                     = (currentBytes >> 8) & 0xFF;
    motorData[4]                     = (currentBytes >> 16) & 0xFF;
    motorData[5]                     = (currentBytes >> 24) & 0xFF;
    motorData[sizeof(motorData) - 1] = crc8(motorData, sizeof(motorData) - 1);
    int8_t state                     = verifyData(motorData, sizeof(motorData), false);
    releaseMutex();
    return state;
}

/**
 * @brief  Get motor speed
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getActualSpeed(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[2], buffer[3], buffer[4], buffer[5]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief   Get motor position
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getActualPosition(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[6], buffer[7], buffer[8], buffer[9]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor current
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getActualCurrent(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[10], buffer[11], buffer[12], buffer[13]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor mode
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getMode(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[14];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor status
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getStatus(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[15];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor error
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getError(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK0_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[16];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor vin
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getActualVin(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK1_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[2], buffer[3], buffer[4], buffer[5]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor temperature
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getActualTemp(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK1_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[6], buffer[7], buffer[8], buffer[9]);
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor encoder
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getEncoder(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK1_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[10], buffer[11], buffer[12], buffer[13]);
    releaseMutex();
    return returnValue;
}

/**
 * @brief  Get motor RGB  mode
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getRGBMode(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK1_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[14];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor RGB  rgbBrightness
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getRGBBrightness(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK1_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[15];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get  motor speed PID
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getSpeedPID(uint8_t id, double *speedPID)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK2_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    speedPID[0]         = handleValue(buffer[2], buffer[3], buffer[4], buffer[5]) / 100000.0f;
    speedPID[1]         = handleValue(buffer[6], buffer[7], buffer[8], buffer[9]) / 10000000.0f;
    speedPID[2]         = handleValue(buffer[10], buffer[11], buffer[12], buffer[13]) / 100000.0f;
    releaseMutex();
    return state;
}

/**
 * @brief  Get motor RGB  mode
 * @param id  Motor equipment id   Value range:0~255
 */
int8_t UnitRoller485::getRGB(uint8_t id, uint8_t *rgbValues)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK2_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    // Assuming buffer indices for R, G, and B values are 16, 15, and 14 respectively
    rgbValues[0] = buffer[16];  // R value
    rgbValues[1] = buffer[15];  // G value
    rgbValues[2] = buffer[14];  // B value
    releaseMutex();
    return state;
}

/**
 * @brief  Get  motor position PID
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getPositionPID(uint8_t id, double *positionPID)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK3_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    positionPID[0]      = handleValue(buffer[2], buffer[3], buffer[4], buffer[5]) / 100000.0f;
    positionPID[1]      = handleValue(buffer[6], buffer[7], buffer[8], buffer[9]) / 10000000.0f;
    positionPID[2]      = handleValue(buffer[10], buffer[11], buffer[12], buffer[13]) / 100000.0f;
    releaseMutex();
    return state;
}
/**
 * @brief  Get motor id
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int32_t UnitRoller485::getMotorId(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK3_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[14];
    releaseMutex();
    return returnValue;
}

/**
 * @brief  Get motor baudrate
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getBaudRate(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK3_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[15];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  Get motor button
 * @param id  Motor equipment id   Value range:0~255
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::getButton(uint8_t id)
{
    acquireMutex();
    memset(Readback, 0, sizeof(Readback));
    Readback[0]                    = ROLLER485_READ_BACK3_CMD;
    Readback[1]                    = id;
    Readback[sizeof(Readback) - 1] = crc8(Readback, sizeof(Readback) - 1);
    int8_t state                   = verifyData(Readback, sizeof(Readback), false);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[16];
    releaseMutex();
    return returnValue;
}

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
int8_t UnitRoller485::readI2c(uint8_t id, uint8_t address, uint8_t dataLen, uint8_t *readBuffer)
{
    acquireMutex();
    memset(readI2cNum1, 0, sizeof(readI2cNum1));
    readI2cNum1[0]                       = ROLLER485_READ_I2C_DATA2_CMD;
    readI2cNum1[1]                       = id;
    readI2cNum1[2]                       = address;
    readI2cNum1[3]                       = dataLen;
    readI2cNum1[sizeof(readI2cNum1) - 1] = crc8(readI2cNum1, sizeof(readI2cNum1) - 1);
    // Send data
    if (!sendData(reinterpret_cast<const char *>(readI2cNum1), sizeof(readI2cNum1))) {
        releaseMutex();
        return ROLLER485_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER485_SERIAL_TIMEOUT;
    }
    int8_t state = verifyResponse(buffer, bytesRead, readI2cNum1[0] + 0x10, true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    for (uint8_t i = 0; i < bytesRead; i++) {
        readBuffer[i] = buffer[i];
    }
    releaseMutex();
    return bytesRead;
}

/**
 * @brief reads data from the I2C device
 *
 * Reads data from the I2C device with the specified ID, address, address length, register address, and data length, and
 * stores the read data into the given read buffer.
 *
 * @param id ID of the I2C device
 * @param address Base address of the I2C device
 * @param addressLen The length of the address (usually 1 or 2, depending on the register address structure of the I2C
 * device)
 * @param regByte0 High byte of register address (if addressLen is 2, otherwise ignored)
 * @param regByte1 Low byte of register address
 * @param dataLen Length of data to be read
 * @param readBuffer Buffer used to store the read data
 *
 * @return The number of bytes read statefully, and an error code if an error occurs
 */
int8_t UnitRoller485::readI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1,
                              uint8_t dataLen, uint8_t *readBuffer)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[3]                       = addressLen;
    readI2cNum2[4]                       = regByte0;
    readI2cNum2[5]                       = regByte1;
    readI2cNum2[6]                       = dataLen;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    // Send data
    if (!sendData(reinterpret_cast<const char *>(readI2cNum2), sizeof(readI2cNum2))) {
        releaseMutex();
        return ROLLER485_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER485_SERIAL_TIMEOUT;
    }
    int8_t state = verifyResponse(buffer, bytesRead, readI2cNum2[0] + 0x10, true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    for (uint8_t i = 0; i < bytesRead; i++) {
        readBuffer[i] = buffer[i];
    }
    releaseMutex();
    return bytesRead;
}

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
int8_t UnitRoller485::readOutput(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[8];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the mode from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the mode value and return an error code if an error occurs
 */
int8_t UnitRoller485::readMode(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[9];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the status from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the status value and return an error code if an error occurs
 */
int8_t UnitRoller485::readStatus(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[20];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the error from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device  Value range:0~127
 * @return Read the error value and return an error code if an error occurs
 */
int8_t UnitRoller485::readError(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[21];
    releaseMutex();
    return returnValue;
}
/**
 * @brief   reads the button from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the button value and return an error code if an error occurs
 */
int8_t UnitRoller485::readButton(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[22];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the stall protection from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the stall protection value and return an error code if an error occurs
 */
int8_t UnitRoller485::readStallProtection(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[23];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor id from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the id  value and return an error code if an error occurs
 */
int8_t UnitRoller485::readMotorId(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_DISPOSTION_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[8];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor baud rate from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the baud rate value and return an error code if an error occurs
 */
int8_t UnitRoller485::readBaudRate(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_DISPOSTION_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[9];
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor rgbbrightness from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the rgbbrightness value and return an error code if an error occurs
 */
int8_t UnitRoller485::readRgbBrightness(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_DISPOSTION_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[10];
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor speed from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the speed  value and return an error code if an error occurs
 */
int32_t UnitRoller485::readSpeed(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_SPEEDMODE_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor speed current from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the speed current value and return an error code if an error occurs
 */
int32_t UnitRoller485::readSpeedCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_SPEEDMODECURRENT_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100.0f;
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor speed PID from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @param speedPID Pointer to type double to store the read speedPID parameter
 *
 * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
 * returned
 */
int8_t UnitRoller485::readSpeedPID(uint8_t id, uint8_t address, double *speedPID)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_SPEEDMODEPID_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    speedPID[0] = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100000.0f;
    speedPID[1] = handleValue(buffer[12], buffer[13], buffer[14], buffer[15]) / 10000000.0f;
    speedPID[2] = handleValue(buffer[16], buffer[17], buffer[18], buffer[19]) / 100000.0f;
    releaseMutex();
    return state;
}

/**
 * @brief  reads the motor position from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the postion value and return an error code if an error occurs
 */
int32_t UnitRoller485::readPosition(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_POSITIONMODE_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor position current from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the position current  value and return an error code if an error occurs
 */
int32_t UnitRoller485::readPositionCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_POSTIONMODECURRENT_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor postion PID from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @param positionPID Pointer to type double to store the read postionPID parameter
 *
 * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
 * returned
 */
int8_t UnitRoller485::readPositionPID(uint8_t id, uint8_t address, double *positionPID)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_POSITIONMODEPID_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    positionPID[0] = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100000.0f;
    positionPID[1] = handleValue(buffer[12], buffer[13], buffer[14], buffer[15]) / 10000000.0f;
    positionPID[2] = handleValue(buffer[16], buffer[17], buffer[18], buffer[19]) / 100000.0f;
    releaseMutex();
    return state;
}
/**
 * @brief  reads the motor current from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the current value and return an error code if an error occurs
 */
int32_t UnitRoller485::readCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_CURRENTMODE_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[8], buffer[9], buffer[10], buffer[11]) / 100.0f;
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor rgb from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @param rgbValues Pointer to type double to store the read rgbValues parameter
 *
 * @return Indicates the operation status code. 1 is returned if the operation succeeds. Otherwise, an error code is
 * returned
 */
int8_t UnitRoller485::readRGB(uint8_t id, uint8_t address, uint8_t *rgbValues)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    // Assuming buffer indices for R, G, and B values are 16, 15, and 14 respectively
    rgbValues[0] = buffer[10];  // R value
    rgbValues[1] = buffer[9];   // G value
    rgbValues[2] = buffer[8];   // B value
    rgbValues[3] = buffer[11];  // rgb mode value

    return state;
}

/**
 * @brief  reads the motor vin from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the vin value and return an error code if an error occurs
 */
int32_t UnitRoller485::readVin(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[12], buffer[13], buffer[14], buffer[15]) / 100.0f;
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor temp from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the temp value and return an error code if an error occurs
 */
int32_t UnitRoller485::readTemp(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[16], buffer[17], buffer[18], buffer[19]);
    releaseMutex();
    return returnValue;
}
/**
 * @brief  reads the motor encoder from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the encoder value and return an error code if an error occurs
 */
int32_t UnitRoller485::readEncoder(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = handleValue(buffer[20], buffer[21], buffer[22], buffer[23]);
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor version from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the version value and return an error code if an error occurs
 */
int8_t UnitRoller485::readVersion(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_VERSION_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = buffer[8];
    releaseMutex();
    return returnValue;
}

/**
 * @brief  reads the motor i2caAddress from the I2C device
 * @param id  Motor equipment id   Value range:0~255
 * @param address Address of the I2C device Value range:0~127
 * @return Read the i2caAddress value and return an error code if an error occurs
 */
int8_t UnitRoller485::readI2cAddress(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = ROLLER485_I2C_READ_I2CADDRESS;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    int32_t returnValue = 0;
    returnValue         = buffer[8];
    releaseMutex();
    return returnValue;
}
/**
 * @brief  wirte i2c
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address
 * @param data   Write data
 * @param dataLen  Write data len
 * @param stopBit  Stop bit  Usually false
 */
int8_t UnitRoller485::writeI2c(uint8_t id, uint8_t address, uint8_t *data, uint8_t dataLen, bool stopBit)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0] = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1] = id;
    writeI2cNum[2] = address;
    writeI2cNum[3] = dataLen;
    writeI2cNum[4] = stopBit;
    for (int i = 0; i < dataLen; i++) {
        writeI2cNum[8 + i] = data[i];
    }
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}
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
int8_t UnitRoller485::writeI2c(uint8_t id, uint8_t address, uint8_t addressLen, uint8_t regByte0, uint8_t regByte1,
                               uint8_t *data, uint8_t dataLen)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0] = ROLLER485_WRITE_I2C_DATA1_CMD;
    writeI2cNum[1] = id;
    writeI2cNum[2] = address;
    writeI2cNum[3] = addressLen;
    writeI2cNum[4] = regByte0;
    writeI2cNum[5] = regByte1;
    writeI2cNum[6] = dataLen;
    for (int i = 0; i < dataLen; i++) {
        writeI2cNum[8 + i] = data[i];
    }
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write MotorConfig
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address
 * @param motorEn    motor enable or off
 * @param mode    Mode setting   1: Speed Mode 2: Position Mode 3: Current
 * Mode 4. Encoder Mode
 * @param removeProtection  Release Jam protection: Send 1 to unprotect
 * @param buttonEn  0: Off; 1: Press and hold for 5S to switch modes in running
 * mode.
 * @param stallProtection   Motor Stall Protection:: 0,Disable; 1, Enable
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeMotorConfig(uint8_t id, uint8_t address, bool motorEn, uint8_t mode, bool removeProtection,
                                       bool buttonEn, bool stallProtection)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_MOTORCONFIG_REG;
    writeI2cNum[9]                       = motorEn;
    writeI2cNum[10]                      = mode;
    writeI2cNum[11]                      = removeProtection;
    writeI2cNum[12]                      = buttonEn;
    writeI2cNum[13]                      = stallProtection;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write_disposition
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param motorId     Value range:0~255
 * @param baudRate    BPS: 0,115200bps; 1, 19200bps; 2, 9600bps;    Default
 value：0
 * @param rgbBrightness   Value range:0~100
 * @return The result or error code of the response validation
*/
int8_t UnitRoller485::writeDisposition(uint8_t id, uint8_t address, uint8_t motorId, uint8_t baudRate,
                                       uint8_t rgbBrightness)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_DISPOSTION_REG;
    writeI2cNum[9]                       = motorId;
    writeI2cNum[10]                      = baudRate;
    writeI2cNum[11]                      = rgbBrightness;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write_speedMode
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address  Value range:0~127
 * @param speed    Speed  Value:-21000000 ~21000000
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeSpeedMode(uint8_t id, uint8_t address, int32_t speed)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    acquireMutex();
    speed              = constrain(speed, min, max);
    int32_t speedBytes = (int32_t)(speed * 100);
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_SPEEDMODE_REG;
    writeI2cNum[9]                       = speedBytes & 0xFF;
    writeI2cNum[10]                      = (speedBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (speedBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (speedBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write speedMode  current
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param current    current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeSpeedModeCurrent(uint8_t id, uint8_t address, int32_t current)
{
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    current              = constrain(current, currentMin, currentMax);
    int32_t currentBytes = (int32_t)(current * 100);
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_SPEEDMODECURRENT_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
int8_t UnitRoller485::writeSpeedModePID(uint8_t id, uint8_t address, uint32_t speedP, uint32_t speedI, uint32_t speedD)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_SPEEDMODEPID_REG;
    writeI2cNum[9]                       = speedP & 0xFF;
    writeI2cNum[10]                      = (speedP >> 8) & 0xFF;
    writeI2cNum[11]                      = (speedP >> 16) & 0xFF;
    writeI2cNum[12]                      = (speedP >> 24) & 0xFF;
    writeI2cNum[13]                      = speedI & 0xFF;
    writeI2cNum[14]                      = (speedI >> 8) & 0xFF;
    writeI2cNum[15]                      = (speedI >> 16) & 0xFF;
    writeI2cNum[16]                      = (speedI >> 24) & 0xFF;
    writeI2cNum[17]                      = speedD & 0xFF;
    writeI2cNum[18]                      = (speedD >> 8) & 0xFF;
    writeI2cNum[19]                      = (speedD >> 16) & 0xFF;
    writeI2cNum[20]                      = (speedD >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write positionMode
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param position   position  Value:-21000000 ~21000000
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writePositionMode(uint8_t id, uint8_t address, int32_t position)
{
    static const int32_t min = -21000000;
    static const int32_t max = 21000000;
    acquireMutex();
    position              = constrain(position, min, max);
    int32_t positionBytes = (int32_t)(position * 100);
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_POSITIONMODE_REG;
    writeI2cNum[9]                       = positionBytes & 0xFF;
    writeI2cNum[10]                      = (positionBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (positionBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (positionBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write positionMode current
 * @param address Device address   Value range:0~127
 * @param id  Motor equipment id   Value range:0~255
 * @param current    current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writePositionModeCurrent(uint8_t id, uint8_t address, int32_t current)
{
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    current              = constrain(current, currentMin, currentMax);
    int32_t currentBytes = (int32_t)(current * 100);
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_POSTIONMODECURRENT_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
int8_t UnitRoller485::writePositionModePID(uint8_t id, uint8_t address, uint32_t positionP, uint32_t positionI,
                                           uint32_t positionD)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_POSITIONMODEPID_REG;
    writeI2cNum[9]                       = positionP & 0xFF;
    writeI2cNum[10]                      = (positionP >> 8) & 0xFF;
    writeI2cNum[11]                      = (positionP >> 16) & 0xFF;
    writeI2cNum[12]                      = (positionP >> 24) & 0xFF;
    writeI2cNum[13]                      = positionI & 0xFF;
    writeI2cNum[14]                      = (positionI >> 8) & 0xFF;
    writeI2cNum[15]                      = (positionI >> 16) & 0xFF;
    writeI2cNum[16]                      = (positionI >> 24) & 0xFF;
    writeI2cNum[17]                      = positionD & 0xFF;
    writeI2cNum[18]                      = (positionD >> 8) & 0xFF;
    writeI2cNum[19]                      = (positionD >> 16) & 0xFF;
    writeI2cNum[20]                      = (positionD >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write currentMode
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param current   current  Value:-1200 ~1200
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeCurrentMode(uint8_t id, uint8_t address, int32_t current)
{
    static const int32_t currentMin = -1200;
    static const int32_t currentMax = 1200;
    acquireMutex();
    current              = constrain(current, currentMin, currentMax);
    int32_t currentBytes = (uint32_t)(current * 100);
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_CURRENTMODE_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
int8_t UnitRoller485::writeSetRGB(uint8_t id, uint8_t address, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB,
                                  uint8_t rgbMode)
{
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    writeI2cNum[9]                       = rgbB;
    writeI2cNum[10]                      = rgbG;
    writeI2cNum[11]                      = rgbR;
    writeI2cNum[11]                      = rgbMode;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write encoder
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param encoder    encoder  uint32_t
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeEncoderMode(uint8_t id, uint8_t address, int32_t encoder)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_RGB_VIN_TEMP_ENCODER_REG;
    writeI2cNum[9]                       = encoder & 0xFF;
    writeI2cNum[10]                      = (encoder >> 8) & 0xFF;
    writeI2cNum[11]                      = (encoder >> 16) & 0xFF;
    writeI2cNum[12]                      = (encoder >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

/**
 * @brief  write  i2c id
 * @param id  Motor equipment id   Value range:0~255
 * @param address Device address   Value range:0~127
 * @param saveFlashEn Save to flash: Send 1 save parameters to flash
 * @param newAddress  new address  Value range:0~127
 * @return The result or error code of the response validation
 */
int8_t UnitRoller485::writeI2cId(uint8_t id, uint8_t address, bool saveFlashEn, uint8_t newAddress)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = ROLLER485_I2C_ID_FLASH_REG;
    writeI2cNum[9]                       = saveFlashEn;
    writeI2cNum[10]                      = newAddress;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}