#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;  // Create a UNIT_ROLLERCAN object
#define motorID              0xA8
#define ultrasonicI2cAddress 0x57
uint8_t data[8] = {ultrasonicI2cAddress, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void setup()
{
    M5.begin();
    Serial.begin(115200);
    Serial.println("ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
}

void loop()
{
    rollercan.writeMotorRawParameter(motorID, ultrasonicI2cAddress, 0x01, 0, 0, 0x00);
    data[1] = 3;
    // This delay is necessary to ensure that there is enough time for the device to turn on the ranging
    delay(300);
    rollercan.sendData(motorID, ROLLERCAN_READ_I2C_RAW_CMD, 0, data);
    rollercan.receiveData();
    float distance =
        ((((rollercan.readData[0] * 256.0) + rollercan.readData[1]) * 256.0) + rollercan.readData[2]) / 10000.0;
    if (distance > 450.0) {
        distance = 450.00;
    }
    Serial.printf("distance: %2f cm\n", distance);
    delay(5000);
}
