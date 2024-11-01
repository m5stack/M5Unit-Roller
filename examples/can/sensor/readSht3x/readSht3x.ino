#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;  // Create a UNIT_ROLLERCAN object
#define motorID        0xA8
#define sht3xI2cAddres 0x44
uint8_t data[8] = {sht3xI2cAddres, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup()
{
    M5.begin();
    Serial.begin(115200);
    Serial.println("ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
}

void loop()
{
    rollercan.writeMotorRawParameter(motorID, sht3xI2cAddres, 0x2c, 0, 1, 0x06);
    rollercan.sendData(motorID, ROLLERCAN_READ_I2C_RAW_CMD, 0, data);
    rollercan.receiveData();
    float cTemp    = ((((rollercan.readData[0] * 256.0) + rollercan.readData[1]) * 175) / 65535.0) - 45;
    float humidity = ((((rollercan.readData[3] * 256.0) + rollercan.readData[4]) * 100) / 65535.0);
    Serial.printf("cTemp : %2f \n", cTemp);
    Serial.printf("humidity: %2f %\n", humidity);
    delay(3000);
}