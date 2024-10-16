/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_roller485.hpp"

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
        return ROLLER_CRC_CHECK_FAIL;  // CRC check failed
    }
    // Check if response matches expected
    if (responseBuffer[0] != expectedResponse) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Unexpected response received.\n");
#else
#endif
        return ROLLER_UNEXPECTED_RESPONSE;  // Unexpected response
    }
    if (verifyResponse) {
        if (responseBuffer[2] != 1) {
#if defined UNIT_ROLLER_DEBUG
            serialPrint("I2C write or read failed\n");
#else
#endif
            return ROLLER_WRITE_FAILED;
        }
    }
    return ROLLER_WRITE_SUCCESS;
}

int8_t UnitRoller485::verifyData(uint8_t *data, size_t length, bool verify)
{
    // Send data
    if (!sendData(reinterpret_cast<const char *>(data), length)) {
        releaseMutex();
        return ROLLER_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER_SERIAL_TIMEOUT;
    }
    int response = verifyResponse(buffer, bytesRead, data[0] + 0x10, verify);
    releaseMutex();
    return response;
}

void UnitRoller485::begin(HardwareSerial *serial, unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin,
                          int8_t dirPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd)
{
    serialPort   = serial;
    this->dirPin = dirPin;
    serialPort->begin(baud, SERIAL_8N1, rxPin, txPin, invert, timeout_ms, rxfifo_full_thrhd);
    if (dirPin != -1) {
        pinMode(dirPin, OUTPUT);
        digitalWrite(dirPin, HIGH);
    }
}

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
    if (dirPin != -1) {
        digitalWrite(dirPin, HIGH);
    }
    size_t bytesSent = serialPort->write(reinterpret_cast<const uint8_t *>(data), length);
    if (bytesSent != length) {
        return false;
    }
    if (dirPin != -1) {
        digitalWrite(dirPin, LOW);
    }
    return true;
}

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

int8_t UnitRoller485::setMode(uint8_t id, roller_mode_t mode)
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

int8_t UnitRoller485::setRGB(uint8_t id, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB, uint8_t rgbBrightness,
                             roller_rgb_t rgbMode)
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

int8_t UnitRoller485::setBaudRate(uint8_t id, roller_bps_t baudRate)
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
        return ROLLER_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER_SERIAL_TIMEOUT;
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
        return ROLLER_SERIAL_SEND_FAILURE;  // Failed to send data
    }
    size_t bytesRead = readData();
    if (bytesRead <= 0) {
#if defined UNIT_ROLLER_DEBUG
        serialPrint("Timeout occurred while waiting for response.\n");
#else
#endif
        releaseMutex();
        return ROLLER_SERIAL_TIMEOUT;
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

int8_t UnitRoller485::readOutput(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readMode(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readStatus(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readError(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readButton(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readRangeProtection(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
    readI2cNum2[6]                       = ROLLER485_I2C_DATA_LEN;
    readI2cNum2[sizeof(readI2cNum2) - 1] = crc8(readI2cNum2, sizeof(readI2cNum2) - 1);
    int8_t state                         = verifyData(readI2cNum2, sizeof(readI2cNum2), true);
    if (state != 1) {
        releaseMutex();
        return state;
    }
    uint8_t returnValue = 0;
    returnValue         = buffer[18];
    releaseMutex();
    return returnValue;
}

int8_t UnitRoller485::readStallProtection(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_OUTPUT_REG;
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

int8_t UnitRoller485::readMotorId(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_ID_REG;
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

int8_t UnitRoller485::readBaudRate(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_ID_REG;
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

int8_t UnitRoller485::readRgbBrightness(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_ID_REG;
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

int32_t UnitRoller485::readSpeed(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_SPEED_REG;
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

int32_t UnitRoller485::readSpeedCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_SPEED_MAX_CURRENT_REG;
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

int32_t UnitRoller485::readSpeedReadback(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_SPEED_READBACK_REG;
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

int8_t UnitRoller485::readSpeedPID(uint8_t id, uint8_t address, double *speedPID)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_SPEED_PID_REG;
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

int32_t UnitRoller485::readPosition(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_POS_REG;
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

int32_t UnitRoller485::readPositionCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_POS_MAX_CURRENT_REG;
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

int32_t UnitRoller485::readPositionReadback(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_POS_READBACK_REG;
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

int8_t UnitRoller485::readPositionPID(uint8_t id, uint8_t address, double *positionPID)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_POS_PID_REG;
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

int32_t UnitRoller485::readCurrent(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_CURRENT_REG;
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

int32_t UnitRoller485::readCurrentReadback(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_CURRENT_READBACK_REG;
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

int8_t UnitRoller485::readRGB(uint8_t id, uint8_t address, uint8_t *rgbValues)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_RGB_REG;
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

int32_t UnitRoller485::readVin(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_RGB_REG;
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

int32_t UnitRoller485::readTemp(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_RGB_REG;
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

int32_t UnitRoller485::readEncoder(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_RGB_REG;
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

int8_t UnitRoller485::readVersion(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_FIRMWARE_VERSION_REG;
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

int8_t UnitRoller485::readI2cAddress(uint8_t id, uint8_t address)
{
    acquireMutex();
    memset(readI2cNum2, 0, sizeof(readI2cNum2));
    readI2cNum2[0]                       = ROLLER485_READ_I2C_DATA1_CMD;
    readI2cNum2[1]                       = id;
    readI2cNum2[2]                       = address;
    readI2cNum2[4]                       = I2C_ADDRESS_REG;
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

int8_t UnitRoller485::writeMotorConfig(uint8_t id, uint8_t address, bool motorEn, roller_mode_t mode,
                                       bool rangeProtection, bool removeProtection, bool buttonEn, bool stallProtection)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_OUTPUT_REG;
    writeI2cNum[9]                       = motorEn;
    writeI2cNum[10]                      = mode;
    writeI2cNum[11]                      = rangeProtection;
    writeI2cNum[12]                      = removeProtection;
    writeI2cNum[13]                      = buttonEn;
    writeI2cNum[14]                      = stallProtection;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writeDisposition(uint8_t id, uint8_t address, uint8_t motorId, roller_bps_t baudRate,
                                       uint8_t rgbBrightness)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_ID_REG;
    writeI2cNum[9]                       = motorId;
    writeI2cNum[10]                      = baudRate;
    writeI2cNum[11]                      = rgbBrightness;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
    writeI2cNum[8]                       = I2C_SPEED_REG;
    writeI2cNum[9]                       = speedBytes & 0xFF;
    writeI2cNum[10]                      = (speedBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (speedBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (speedBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
    writeI2cNum[8]                       = I2C_SPEED_MAX_CURRENT_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writeSpeedModePID(uint8_t id, uint8_t address, uint32_t speedP, uint32_t speedI, uint32_t speedD)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_SPEED_PID_REG;
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
    writeI2cNum[8]                       = I2C_POS_REG;
    writeI2cNum[9]                       = positionBytes & 0xFF;
    writeI2cNum[10]                      = (positionBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (positionBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (positionBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

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
    writeI2cNum[8]                       = I2C_POS_MAX_CURRENT_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writePositionModePID(uint8_t id, uint8_t address, uint32_t positionP, uint32_t positionI,
                                           uint32_t positionD)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_POS_PID_REG;
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
    writeI2cNum[8]                       = I2C_CURRENT_REG;
    writeI2cNum[9]                       = currentBytes & 0xFF;
    writeI2cNum[10]                      = (currentBytes >> 8) & 0xFF;
    writeI2cNum[11]                      = (currentBytes >> 16) & 0xFF;
    writeI2cNum[12]                      = (currentBytes >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writeSetRGB(uint8_t id, uint8_t address, uint8_t rgbR, uint8_t rgbG, uint8_t rgbB,
                                  roller_rgb_t rgbMode)
{
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_RGB_REG;
    writeI2cNum[9]                       = rgbB;
    writeI2cNum[10]                      = rgbG;
    writeI2cNum[11]                      = rgbR;
    writeI2cNum[12]                      = rgbMode;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writeEncoderMode(uint8_t id, uint8_t address, int32_t encoder)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_DIAL_COUNTER_REG;
    writeI2cNum[9]                       = encoder & 0xFF;
    writeI2cNum[10]                      = (encoder >> 8) & 0xFF;
    writeI2cNum[11]                      = (encoder >> 16) & 0xFF;
    writeI2cNum[12]                      = (encoder >> 24) & 0xFF;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}

int8_t UnitRoller485::writeI2cId(uint8_t id, uint8_t address, bool saveFlashEn, uint8_t newAddress)
{
    acquireMutex();
    memset(writeI2cNum, 0, sizeof(writeI2cNum));
    writeI2cNum[0]                       = ROLLER485_WRITE_I2C_DATA2_CMD;
    writeI2cNum[1]                       = id;
    writeI2cNum[2]                       = address;
    writeI2cNum[3]                       = ROLLER485_I2C_DATA_LEN;
    writeI2cNum[8]                       = I2C_SAVE_FLASH_REG;
    writeI2cNum[9]                       = saveFlashEn;
    writeI2cNum[10]                      = newAddress;
    writeI2cNum[sizeof(writeI2cNum) - 1] = crc8(writeI2cNum, sizeof(writeI2cNum) - 1);
    int8_t state                         = verifyData(writeI2cNum, sizeof(writeI2cNum), true);
    releaseMutex();
    return state;
}