#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;     // Create a UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id      0x00
#define slaveI2cAddress 0x64
uint8_t slaveRgbValues[4];  // Array to store R, G, B ,mode values
double slaveSpeedPID[3];    // Array to P I D values
void setup()
{
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 16, 17, -1, false, 10000UL, 112U);
    // Set the motor mode to speed
    Roller485.setMode(motor485Id, ROLLER_MODE_SPEED);
    // Set speed Speed and current
    Roller485.setSpeedMode(motor485Id, 2400, 1200);
    // Set the motor to enable
    Roller485.setOutput(motor485Id, 0x01);

    // Set the speed mode of the slave motor
    Roller485.writeMotorConfig(motor485Id, slaveI2cAddress, false, ROLLER_MODE_SPEED, true, true, false, true);
    Roller485.writeSpeedMode(motor485Id, slaveI2cAddress, 1000);
    Roller485.writeSpeedModeCurrent(motor485Id, slaveI2cAddress, 1200);
    // set  position
    //  Roller485.writeMotorConfig(motor485Id, slaveI2cAddress, true, 0x02, 0x00, true, true);
    //  Roller485.writePositionMode(motor485Id, slaveI2cAddress, 500000);
    //  Roller485.writePositionModeCurrent(motor485Id, slaveI2cAddress, 500);

    // set current
    //  Roller485.writeMotorConfig(motor485Id, slaveI2cAddress, true, 0x03, 0x00, true, true);
    //  Roller485.writeCurrentMode(motor485Id, slaveI2cAddress, 800);
}

void loop()
{
    int errorCode = Roller485.setMode(motor485Id, ROLLER_MODE_SPEED);
    if (errorCode == 1) {
        // Successful operation
        Serial.println("setMode() successful.");
    } else {
        // Print error code
        Serial.print("setMode() failed with error code: ");
        Serial.println(errorCode);
    }
    delay(1000);

    int8_t output = Roller485.readOutput(motor485Id, slaveI2cAddress);
    Serial.printf("output: %d\n", output);
    delay(1000);
    int8_t mode = Roller485.readMode(motor485Id, slaveI2cAddress);
    Serial.printf("mode: %d\n", mode);
    delay(1000);
    int8_t button = Roller485.readButton(motor485Id, slaveI2cAddress);
    Serial.printf("button: %d\n", button);
    delay(1000);
    int8_t stallProtection = Roller485.readStallProtection(motor485Id, slaveI2cAddress);
    Serial.printf("stallProtection: %d\n", stallProtection);
    delay(1000);
    int8_t status = Roller485.readStatus(motor485Id, slaveI2cAddress);
    Serial.printf("status: %d\n", status);
    delay(1000);
    int8_t error = Roller485.readError(motor485Id, slaveI2cAddress);
    Serial.printf("error: %d\n", error);
    delay(1000);
    int32_t motorId = Roller485.readMotorId(motor485Id, slaveI2cAddress);
    Serial.printf("motorId: %d\n", motorId);
    delay(1000);
    int8_t baudRate = Roller485.readBaudRate(motor485Id, slaveI2cAddress);
    Serial.printf("baudRate: %d\n", baudRate);
    delay(1000);
    int8_t rgbBrightness = Roller485.readRgbBrightness(motor485Id, slaveI2cAddress);
    Serial.printf("rgbBrightness: %d\n", rgbBrightness);
    delay(1000);
    int8_t errorSpeedPID = Roller485.readSpeedPID(motor485Id, slaveI2cAddress, slaveSpeedPID);
    if (errorSpeedPID != 1) {
        Serial.print("getSpeedPID failed with error code: ");
        Serial.println(errorSpeedPID);
    } else {
        printf("speedPID values: P=%.8lf, I=%.8lf, D=%.8lf\n", slaveSpeedPID[0], slaveSpeedPID[1], slaveSpeedPID[2]);
    }

    delay(1000);

    int32_t current = Roller485.readCurrent(motor485Id, slaveI2cAddress);
    Serial.printf("current: %d\n", current);
    delay(1000);
    Roller485.writeSetRGB(motor485Id, slaveI2cAddress, 0xff, 0xff, 0xff, ROLLER_RGB_MODE_USER_DEFINED);
    delay(1000);
    int8_t errorRGB = Roller485.readRGB(motor485Id, slaveI2cAddress, slaveRgbValues);
    if (errorRGB != 1) {
        // Print error code
        Serial.print("getRGB failed with error code: ");
        Serial.println(errorRGB);
    } else {
        printf("RGB values: R=%d, G=%d, B=%d  Mode:%d\n", slaveRgbValues[0], slaveRgbValues[1], slaveRgbValues[2],
               slaveRgbValues[3]);
    }
    Roller485.writeSetRGB(motor485Id, slaveI2cAddress, 0x00, 0x00, 0x00, ROLLER_RGB_MODE_DEFAULT);
    int32_t vin = Roller485.readVin(motor485Id, slaveI2cAddress);
    Serial.printf("vin: %d\n", vin);
    delay(1000);
    int32_t temp = Roller485.readTemp(motor485Id, slaveI2cAddress);
    Serial.printf("temp: %d\n", temp);
    delay(1000);
    int32_t encoder = Roller485.readEncoder(motor485Id, slaveI2cAddress);
    Serial.printf("encoder: %d\n", encoder);
    delay(1000);
    int version = Roller485.readVersion(motor485Id, slaveI2cAddress);
    Serial.printf("version: %d\n", version);
    delay(1000);
    uint8_t i2cAddress = Roller485.readI2cAddress(motor485Id, slaveI2cAddress);
    Serial.printf("i2cAddress: %d\n", i2cAddress);
    delay(1000);

    Roller485.writeMotorConfig(motor485Id, slaveI2cAddress, false, ROLLER_MODE_ENCODER, true, true, false, true);

    Roller485.writeEncoderMode(motor485Id, slaveI2cAddress, 24000);
    Serial.printf("readEncoder: %d\n", Roller485.readEncoder(motor485Id, slaveI2cAddress));
}