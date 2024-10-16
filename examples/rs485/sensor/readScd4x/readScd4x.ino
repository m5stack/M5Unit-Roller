#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;     // Create a UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id      0x00
#define scd4xI2cAddress 0x62
uint8_t readBuffer[128];
static uint8_t scd4xSetup[] = {0x21, 0xB1};
static uint8_t scd4xWrite[] = {0xEC, 0x05};
void setup()
{
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 16, 17, -1, false, 10000UL, 112U);
    // Set the sensor to return data every 5 seconds
    uint8_t errorSetup = Roller485.writeI2c(motor485Id, scd4xI2cAddress, scd4xSetup, sizeof(scd4xSetup), 0);
    if (errorSetup != 1) {
        // Print error code
        Serial.print("writeI2c setup failed with error code: ");
        Serial.println(errorSetup);
    }
}

void loop()
{
    uint8_t errorWrite = Roller485.writeI2c(motor485Id, scd4xI2cAddress, scd4xWrite, sizeof(scd4xWrite), 0);
    if (errorWrite != 1) {
        // Print error code
        Serial.print("writeI2c failed with error code: ");
        Serial.println(errorWrite);
    }
    int result = Roller485.readI2c(motor485Id, scd4xI2cAddress, 0x0C, readBuffer);
    if (result > 0) {
        for (size_t i = 0; i < result; i++) {
            Serial.print("0x");
            Serial.print(readBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\n");
        float co2         = (float)((uint16_t)readBuffer[8] << 8 | readBuffer[9]);
        float temperature = -45 + 175 * (float)((uint16_t)readBuffer[11] << 8 | readBuffer[12]) / 65536;
        float humidity    = 100 * (float)((uint16_t)readBuffer[14] << 8 | readBuffer[15]) / 65536;
        Serial.printf("co2 : %2f\n", co2);
        Serial.printf("temperature : %2f\n", temperature);
        Serial.printf("humidity: %2f\n", humidity);

    } else {
        // Print error code
        Serial.print("readI2c failed with error code: ");
        Serial.println(result);
    }
    delay(5000);
}
