#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;  // Create a UNIT_ROLLERCAN object
#define motorID         0xA8
#define scd4xI2cAddress 0x62
uint8_t data[8] = {scd4xI2cAddress, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void setup()
{
    M5.begin();
    Serial.begin(115200);
    Serial.println("ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
    rollercan.writeMotorRawParameter(motorID, scd4xI2cAddress, 0x21, 0, 1, 0xB1);
}

void loop()
{
    rollercan.writeMotorRawParameter(motorID, scd4xI2cAddress, 0xEC, 0, 1, 0x05);

    rollercan.writeMotorRawParameter(motorID, scd4xI2cAddress, 0x0C, 0, 0, 0x00);
    data[1] = 8;
    rollercan.sendData(motorID, ROLLERCAN_READ_I2C_RAW_CMD, 0, data);

    rollercan.receiveData();
    float co2         = (float)(rollercan.readData[0] << 8 | rollercan.readData[1]);
    float temperature = -45 + 175 * (float)(rollercan.readData[3] << 8 | rollercan.readData[4]) / 65536;
    float humidity    = 100 * (float)(rollercan.readData[6] << 8 | rollercan.readData[7]) / 65536;
    Serial.printf("co2 : %2f\n", co2);
    Serial.printf("temperature : %2f\n", temperature);
    Serial.printf("humidity: %2f\n", humidity);
    delay(5000);
}
