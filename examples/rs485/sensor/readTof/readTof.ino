#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;     // Create a  UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id    0x00
#define tofI2cAddress 0x29
uint8_t readBuffer[128];
static uint8_t tofWrite[] = {0x00, 0x01};
void setup()
{
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 22, 21, false, 10000UL, 112U);
}

void loop()
{
    uint8_t errorWrite = Roller485.writeI2c(motor485Id, tofI2cAddress, tofWrite, sizeof(tofWrite), 0);
    if (errorWrite != 1) {
        // Print error code
        Serial.print("writeI2c failed with error code: ");
        Serial.println(errorWrite);
    }
    int result = Roller485.readI2c(motor485Id, tofI2cAddress, 0x00, 0x14, 0x00, 0x0C, readBuffer);
    if (result > 0) {
        for (size_t i = 0; i < result; i++) {
            Serial.print("0x");
            Serial.print(readBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\n");
        float acnt                    = ((readBuffer[14] & 0xFF) << 8) | (readBuffer[15] & 0xFF);
        float scnt                    = ((readBuffer[16] & 0xFF) << 8) | (readBuffer[17] & 0xFF);
        float dist                    = ((readBuffer[18] & 0xFF) << 8) | (readBuffer[19] & 0xFF);
        int DeviceRangeStatusInternal = ((readBuffer[8] & 0x78) >> 3);
        Serial.printf("acnt : %2f\n", acnt);
        Serial.printf("scnt : %2f\n", scnt);
        Serial.printf("dist : %2f\n", dist);
        Serial.printf("DeviceRangeStatusInternal: %d\n", DeviceRangeStatusInternal);

    } else {
        // Print error code
        Serial.print("readI2c failed with error code: ");
        Serial.println(result);
    }
    delay(3000);
}
