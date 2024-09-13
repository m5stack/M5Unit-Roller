#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;     // Create a UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id      0x00
#define sht3xI2cAddress 0x44
uint8_t readBuffer[128];
static uint8_t sht3xWrite[] = {0x2C, 0x06};
void setup()
{
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 22, 21, false, 10000UL, 112U);
}

void loop()
{
    Roller485.writeI2c(motor485Id, sht3xI2cAddress, sht3xWrite, sizeof(sht3xWrite), 0);
    int result = Roller485.readI2c(motor485Id, sht3xI2cAddress, 0x06, readBuffer);
    if (result > 0) {
        for (size_t i = 0; i < result; i++) {
            Serial.print("0x");
            Serial.print(readBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\n");
        float cTemp = ((((readBuffer[8] * 256.0) + readBuffer[9]) * 175) / 65535.0) - 45;
        // fTemp    = (cTemp * 1.8) + 32;
        float humidity = ((((readBuffer[11] * 256.0) + readBuffer[12]) * 100) / 65535.0);
        Serial.printf("cTemp : %2f \n", cTemp);
        Serial.printf("humidity: %2f %\n", humidity);

    } else {
        // Print error code
        Serial.print("readI2c failed with error code: ");
        Serial.println(result);
    }
    delay(3000);
}
