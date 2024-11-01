#include "unit_rollercan.hpp"
#include <M5Unified.h>
UnitRollerCAN rollercan;  // Create a UNIT_ROLLERCAN object
#define motorID       0xA8
#define tofI2cAddress 0x29
uint8_t data[8] = {tofI2cAddress, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}
void setup()
{
    M5.begin();
    Serial.begin(115200);
    Serial.println("ESP32-Arduino-CAN");
    rollercan.beginCAN(1000000, 16, 17);
}

void loop()
{
    rollercan.writeMotorRawParameter(motorID, tofI2cAddress, 0x00, 0, 1, 0x01);

    rollercan.writeMotorRawParameter(motorID, tofI2cAddress, 0x14, 0, 0, 0x00);
    data[1] = 1;
    rollercan.sendData(motorID, ROLLERCAN_READ_I2C_RAW_CMD, 0, data);

    rollercan.receiveData();
    int DeviceRangeStatusInternal = ((rollercan.readData[0] & 0x78) >> 3);

    rollercan.writeMotorRawParameter(motorID, tofI2cAddress, 0x1A, 0, 0, 0x00);
    data[1] = 6;
    rollercan.sendData(motorID, ROLLERCAN_READ_I2C_RAW_CMD, 0, data);
    rollercan.receiveData();
    uint16_t acnt = makeuint16(rollercan.readData[1], rollercan.readData[0]);
    uint16_t scnt = makeuint16(rollercan.readData[3], rollercan.readData[2]);
    uint16_t dist = makeuint16(rollercan.readData[5], rollercan.readData[4]);
    Serial.print("ambient count: ");
    Serial.println(acnt);
    Serial.print("signal count: ");
    Serial.println(scnt);
    Serial.print("distance: ");
    Serial.println(dist);
    Serial.print("status: ");
    Serial.println(DeviceRangeStatusInternal);
    delay(3000);
}
