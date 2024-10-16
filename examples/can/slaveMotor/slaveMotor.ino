#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;  // Create a UNIT_ROLLERCAN object
#define motorID         0xA8
#define slaveI2cAddress 0x64
uint8_t slaveRgbValues[3];  // Array to store R, G, B ,mode values

void setup()
{
    M5.begin();
    Serial.begin(115200);
    Serial.println("ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
    rollercan.setMode(motorID, ROLLER_MODE_SPEED);
    rollercan.setSpeed(motorID, 1200);
    rollercan.setSpeedCurrent(motorID, 1200);
    rollercan.setOutput(motorID, false);
    delay(1000);
    rollercan.writeMode(motorID, slaveI2cAddress, ROLLER_MODE_ENCODER);
    rollercan.writeEncoder(motorID, slaveI2cAddress, 24000);
    Serial.printf("read encoder:%d\r\n", rollercan.readEncoder(motorID, slaveI2cAddress));
    delay(3000);
    rollercan.writeOutput(motorID, slaveI2cAddress, true);
    rollercan.writeRemoveProtection(motorID, slaveI2cAddress, false);
    rollercan.writeMode(motorID, slaveI2cAddress, ROLLER_MODE_SPEED);
    rollercan.writeSaveFlash(motorID, slaveI2cAddress, true);
    delay(1000);

    rollercan.writeSpeed(motorID, slaveI2cAddress, -2400);
    rollercan.writeSpeedCurrent(motorID, slaveI2cAddress, 1200);
    Serial.printf("readActualSpeed:%d\r\n", rollercan.readActualSpeed(motorID, slaveI2cAddress));
    Serial.printf("readSpeed:%d\r\n", rollercan.readSpeed(motorID, slaveI2cAddress));
    Serial.printf("readSpeedCurrent:%d\r\n", rollercan.readSpeedCurrent(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeOutput(motorID, slaveI2cAddress, false);
    rollercan.writeSpeedKp(motorID, slaveI2cAddress, 15 * 100000);
    Serial.printf("readSpeedKp:%d\r\n", rollercan.readSpeedKp(motorID, slaveI2cAddress));

    rollercan.writeSpeedKi(motorID, slaveI2cAddress, 100000);
    Serial.printf("readSpeedKi:%d\r\n", rollercan.readSpeedKi(motorID, slaveI2cAddress));

    rollercan.writeSpeedKd(motorID, slaveI2cAddress, 400 * 100000);
    Serial.printf("readSpeedKd:%d\r\n", rollercan.readSpeedKd(motorID, slaveI2cAddress));

    delay(3000);

    rollercan.writeMode(motorID, slaveI2cAddress, ROLLER_MODE_SPEED);
    rollercan.writePosition(motorID, slaveI2cAddress, 240000);
    rollercan.writePositionCurrent(motorID, slaveI2cAddress, 1200);
    rollercan.writeOutput(motorID, slaveI2cAddress, true);
    Serial.printf("readActualPosition:%d\r\n", rollercan.readActualPosition(motorID, slaveI2cAddress));

    Serial.printf("readPosition:%d\r\n", rollercan.readPosition(motorID, slaveI2cAddress));

    Serial.printf("readPositionCurrent:%d\r\n", rollercan.readPositionCurrent(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeOutput(motorID, slaveI2cAddress, false);
    rollercan.writePositionKp(motorID, slaveI2cAddress, 15 * 100000);
    Serial.printf("readPositionKp:%d\r\n", rollercan.readPositionKp(motorID, slaveI2cAddress));

    rollercan.writePositionKi(motorID, slaveI2cAddress, 30);
    Serial.printf("readPositionKi:%d\r\n", rollercan.readPositionKi(motorID, slaveI2cAddress));

    rollercan.writePositionKd(motorID, slaveI2cAddress, 100 * 100000);
    Serial.printf("readPositionKd:%d\r\n", rollercan.readSpeedKd(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeMode(motorID, slaveI2cAddress, ROLLER_MODE_ENCODER);
    rollercan.writeCurrent(motorID, slaveI2cAddress, 2400);
    rollercan.writeOutput(motorID, slaveI2cAddress, true);
    Serial.printf("readCurrent:%d\r\n", rollercan.readCurrent(motorID, slaveI2cAddress));

    Serial.printf("readActualCurrent:%d\r\n", rollercan.readActualCurrent(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeOutput(motorID, slaveI2cAddress, false);
    rollercan.writeRGBMode(motorID, slaveI2cAddress, ROLLER_RGB_MODE_USER_DEFINED );
    Serial.printf("readRGBMode:%d\r\n", rollercan.readRGBMode(motorID, slaveI2cAddress));

    rollercan.writeRGBBrightness(motorID, slaveI2cAddress, 100);
    Serial.printf("readRGBBrightness:%d\r\n", rollercan.readRGBBrightness(motorID, slaveI2cAddress));

    rollercan.writeRGB(motorID, slaveI2cAddress, WHITE);
    delay(2000);
    rollercan.readRGB(motorID, slaveI2cAddress, slaveRgbValues);
    Serial.printf("RGB values: R=%d, G=%d, B=%d\n", slaveRgbValues[0], slaveRgbValues[1], slaveRgbValues[2]);

    rollercan.writeRGB(motorID, slaveI2cAddress, RED);
    delay(2000);
    rollercan.writeRGB(motorID, slaveI2cAddress, GREEN);
    delay(2000);
    rollercan.writeRGB(motorID, slaveI2cAddress, BLUE);
    delay(2000);
    rollercan.writeRGBMode(motorID, slaveI2cAddress, ROLLER_RGB_MODE_USER_DEFINED );
    Serial.printf("readActualVin:%d\r\n", rollercan.readActualVin(motorID, slaveI2cAddress));

    Serial.printf("readActualTemp:%d\r\n", rollercan.readActualTemp(motorID, slaveI2cAddress));

    // raw
    rollercan.writeResetStallProtect(motorID, slaveI2cAddress, true);
    rollercan.writeRawMode(motorID, slaveI2cAddress, ROLLER_MODE_ENCODER);
    rollercan.writeRawEncoder(motorID, slaveI2cAddress, 24000);
    Serial.printf("read encoder:%d\r\n", rollercan.readRawEncoder(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, true);
    rollercan.writeRawMode(motorID, slaveI2cAddress, ROLLER_MODE_SPEED);
    rollercan.writeRawFlash(motorID, slaveI2cAddress, true);
    delay(1000);
    rollercan.writeRawSpeed(motorID, slaveI2cAddress, -2400);
    rollercan.writeRawSpeedCurrent(motorID, slaveI2cAddress, 1200);
    Serial.printf("readRawActualSpeed:%d\r\n", rollercan.readRawActualSpeed(motorID, slaveI2cAddress));

    Serial.printf("readRawSpeed:%d\r\n", rollercan.readRawSpeed(motorID, slaveI2cAddress));

    Serial.printf("readRawSpeedCurrent:%d\r\n", rollercan.readRawSpeedCurrent(motorID, slaveI2cAddress));

    delay(3000);

    delay(3000);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, false);
    rollercan.writeRawSpeedKp(motorID, slaveI2cAddress, 15 * 100000);
    Serial.printf("readRawSpeedKp:%d\r\n", rollercan.readRawSpeedKp(motorID, slaveI2cAddress));

    rollercan.writeRawSpeedKi(motorID, slaveI2cAddress, 100000);
    Serial.printf("readSpeedKi:%d\r\n", rollercan.readRawSpeedKi(motorID, slaveI2cAddress));

    rollercan.writeRawSpeedKd(motorID, slaveI2cAddress, 400 * 100000);
    Serial.printf("readSpeedKd:%d\r\n", rollercan.readRawSpeedKd(motorID, slaveI2cAddress));

    delay(3000);

    rollercan.writeRawMode(motorID, slaveI2cAddress, ROLLER_MODE_SPEED);
    rollercan.writeRawPosition(motorID, slaveI2cAddress, 240000);
    rollercan.writeRawPositionCurrent(motorID, slaveI2cAddress, 1200);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, true);
    Serial.printf("readRawActualPosition:%d\r\n", rollercan.readRawActualPosition(motorID, slaveI2cAddress));

    Serial.printf("readRawPosition:%d\r\n", rollercan.readRawPosition(motorID, slaveI2cAddress));

    Serial.printf("readRawPositionCurrent:%d\r\n", rollercan.readRawPositionCurrent(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, false);
    rollercan.writeRawPositionKp(motorID, slaveI2cAddress, 15 * 100000);
    Serial.printf("readRawPositionKp:%d\r\n", rollercan.readRawPositionKp(motorID, slaveI2cAddress));

    rollercan.writeRawPositionKi(motorID, slaveI2cAddress, 30);
    Serial.printf("readRawPositionKi:%d\r\n", rollercan.readRawPositionKi(motorID, slaveI2cAddress));

    rollercan.writeRawPositionKd(motorID, slaveI2cAddress, 100 * 100000);
    Serial.printf("readPositionKd:%d\r\n", rollercan.readRawSpeedKd(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeRawMode(motorID, slaveI2cAddress, ROLLER_MODE_ENCODER);
    rollercan.writeRawCurrent(motorID, slaveI2cAddress, 2400);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, true);
    Serial.printf("readRawCurrent:%d\r\n", rollercan.readRawCurrent(motorID, slaveI2cAddress));

    Serial.printf("readRawActualCurrent:%d\r\n", rollercan.readRawActualCurrent(motorID, slaveI2cAddress));

    delay(3000);
    rollercan.writeRawOutput(motorID, slaveI2cAddress, false);
    rollercan.writeRawRGBMode(motorID, slaveI2cAddress, ROLLER_RGB_MODE_USER_DEFINED );
    delay(1000);
    Serial.printf("readRawRGBMode:%d\r\n", rollercan.readRawRGBMode(motorID, slaveI2cAddress));

    rollercan.writeRawRGBBrightness(motorID, slaveI2cAddress, 100);
    Serial.printf("readRawRGBBrightness:%d\r\n", rollercan.readRawRGBBrightness(motorID, slaveI2cAddress));

    rollercan.writeRawRGB(motorID, slaveI2cAddress, WHITE);
    delay(2000);
    rollercan.readRawRGB(motorID, slaveI2cAddress, slaveRgbValues);
    Serial.printf("RGB values: R=%d, G=%d, B=%d\n", slaveRgbValues[0], slaveRgbValues[1], slaveRgbValues[2]);

    rollercan.writeRawRGB(motorID, slaveI2cAddress, RED);
    delay(2000);
    Serial.printf("readRawRGBMode:%d\r\n", rollercan.readRawRGBMode(motorID, slaveI2cAddress));
    rollercan.writeRawRGB(motorID, slaveI2cAddress, GREEN);
    delay(2000);
    rollercan.writeRawRGB(motorID, slaveI2cAddress, BLUE);
    delay(2000);
    rollercan.writeRawRGBMode(motorID, slaveI2cAddress, ROLLER_RGB_MODE_DEFAULT );
    Serial.printf("readActualVin:%d\r\n", rollercan.readRawActualVin(motorID, slaveI2cAddress));
    Serial.printf("readActualTemp:%d\r\n", rollercan.readRawActualTemp(motorID, slaveI2cAddress));
    Serial.printf("FirmwareVersion:%d\r\n", rollercan.readRawFirmwareVersion(motorID, slaveI2cAddress));
    rollercan.writeRawI2CAddress(motorID, slaveI2cAddress, 0x23);
    delay(2000);
    rollercan.writeRawI2CAddress(motorID, 0x23, slaveI2cAddress);
    Serial.printf("I2CAddress:%d\r\n", rollercan.readRawI2CAddress(motorID, slaveI2cAddress));

    rollercan.writeStallProtect(motorID, slaveI2cAddress, true);
    Serial.printf("readStallProtect:%d\r\n", rollercan.readStallProtect(motorID, slaveI2cAddress));

    rollercan.writeButton(motorID, slaveI2cAddress, true);
    Serial.printf("readButton:%d\r\n", rollercan.readButton(motorID, slaveI2cAddress));

    Serial.printf("readErrorCode:%d\r\n", rollercan.readErrorCode(motorID, slaveI2cAddress));

    Serial.printf("readSysStatus:%d\r\n", rollercan.readSysStatus(motorID, slaveI2cAddress));
    rollercan.writeMotorID(motorID, slaveI2cAddress, 15);
    delay(1000);
    Serial.printf("readMotorID:%d\r\n", rollercan.readMotorID(motorID, slaveI2cAddress));

    rollercan.writeBPS(motorID, slaveI2cAddress, ROLLER_BPS_CAN_500000);
    Serial.printf("readBPS:%d\r\n", rollercan.readBPS(motorID, slaveI2cAddress));
    rollercan.writeRawFlash(motorID, slaveI2cAddress, true);
}

void loop()
{
    delay(1000);
}