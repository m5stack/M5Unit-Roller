#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;       // Create a UNIT_ROLLERCAN object
const uint8_t motorID = 0xA8;  

uint8_t slaveRgbValues[3];  // Array to store R, G, B ,mode values
void setup()
{
    M5.begin();

    Serial.begin(115200);
    Serial.println("Basic Demo - ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
    rollercan.setMode(motorID, ROLLER_MODE_SPEED);
    Serial.printf("mode:%d\r\n", rollercan.getMode(motorID));

    rollercan.setSaveFlash(motorID);
    rollercan.setSpeedCurrent(motorID, 500);
    Serial.printf("SpeedCurrent:%d\r\n", rollercan.getSpeedCurrent(motorID));

    rollercan.setOutput(motorID, true);
    Serial.printf("Output:%d\r\n", rollercan.getOutput(motorID));

    rollercan.setSpeed(motorID, -500);
    Serial.printf("getSpeed:%d\r\n", rollercan.getSpeed(motorID));

    delay(3000);
    rollercan.setSpeed(motorID, -2400);
    Serial.printf("getActualSpeed:%d\r\n", rollercan.getActualSpeed(motorID));

    delay(3000);
    rollercan.setOutput(motorID, false);
    rollercan.setSpeedKp(motorID, 15 * 100000);
    Serial.printf("getSpeedKp:%d\r\n", rollercan.getSpeedKp(motorID));

    rollercan.setSpeedKi(motorID, 100000);
    Serial.printf("getSpeedKi:%d\r\n", rollercan.getSpeedKi(motorID));

    rollercan.setSpeedKd(motorID, 400 * 100000);
    Serial.printf("getSpeedKd:%d\r\n", rollercan.getSpeedKd(motorID));

    delay(3000);
    // position
    rollercan.setMode(motorID, ROLLER_MODE_SPEED);
    rollercan.setPositionCurrent(motorID, 1200);
    Serial.printf("PositionCurrent:%d\r\n", rollercan.getPositionCurrent(motorID));
    rollercan.setOutput(motorID, true);
    rollercan.setPosition(motorID, 240000);
    Serial.printf("getPosition:%d\r\n", rollercan.getPosition(motorID));

    Serial.printf("getActualPosition:%d\r\n", rollercan.getActualPosition(motorID));

    delay(3000);
    rollercan.setPositionKp(motorID, 15 * 100000);
    Serial.printf("getPositionKp:%d\r\n", rollercan.getPositionKp(motorID));

    rollercan.setPositionKi(motorID, 30);
    Serial.printf("getPositionKi:%d\r\n", rollercan.getPositionKi(motorID));

    rollercan.setPositionKd(motorID, 100 * 100000);
    Serial.printf("getPositionKd:%d\r\n", rollercan.getPositionKd(motorID));

    delay(3000);

    rollercan.setMotorID(motorID, 0x64);
    delay(2000);
    rollercan.setMotorID(0x64, motorID);
    delay(1000);
    rollercan.setRGBMode(motorID, ROLLER_RGB_MODE_USER_DEFINED );
    Serial.printf("getRGBMode:%d\r\n", rollercan.getRGBMode(motorID));

    rollercan.setRGBBrightness(motorID, 100);
    Serial.printf("getRGBBrightness:%d\r\n", rollercan.getRGBBrightness(motorID));

    rollercan.setRGB(motorID, WHITE);
    delay(2000);
    rollercan.getRGB(motorID, slaveRgbValues);
    Serial.printf("RGB values: R=%d, G=%d, B=%d\n", slaveRgbValues[0], slaveRgbValues[1], slaveRgbValues[2]);

    rollercan.setRGB(motorID, RED);
    delay(2000);
    rollercan.setRGB(motorID, GREEN);
    delay(2000);
    rollercan.setRGB(motorID, BLUE);
    delay(2000);
    rollercan.setRGBMode(motorID, ROLLER_RGB_MODE_DEFAULT );
    rollercan.setMode(motorID, ROLLER_MODE_ENCODER);
    rollercan.setCurrent(motorID, 1200);
    Serial.printf("getCurrent:%d\r\n", rollercan.getCurrent(motorID));

    Serial.printf("getActualCurrent:%d\r\n", rollercan.getActualCurrent(motorID));

    delay(3000);
    rollercan.setOutput(motorID, false);
    delay(1000);
    rollercan.setMode(motorID, ROLLER_MODE_ENCODER);
    delay(1000);
    // Serial.printf("res:%d\r\n",rollercan.setEncoder(motorID,12000));
    Serial.printf("res:%d\r\n", rollercan.setEncoder(motorID, 120000));

    delay(1000);
    // Serial.printf("encoder:%d\r\n", rollercan.getEncoder(motorID));
    Serial.printf("encoder:%d\r\n", rollercan.getEncoder(motorID));

    Serial.printf("getActualVin:%d\r\n", rollercan.getActualVin(motorID));

    Serial.printf("getActualTemp:%d\r\n", rollercan.getActualTemp(motorID));
}

void loop()
{
    delay(10000);
}