#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;         // Create a UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id              0x00
#define ultrasonicI2cAddress 0x57
static uint8_t ultrasonicWrite[] = {0x01, 0x01};
uint8_t readBuffer[128];

void setup() {
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 16, 17, false, 10000UL, 112U);
}

void loop() {
    uint8_t errorWrite = Roller485.writeI2c(motor485Id, ultrasonicI2cAddress,ultrasonicWrite, sizeof(ultrasonicWrite), 1);
    if (errorWrite != 1) {
        // Print error code
        Serial.print("writeI2c failed with error code: ");
        Serial.println(errorWrite);
    }
    int result = Roller485.readI2c(motor485Id, ultrasonicI2cAddress, 0x03, readBuffer);
    if (result > 0) {
        for (size_t i = 0; i < result; i++) {
            Serial.print("0x");
            Serial.print(readBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.print("\n");
        float distance=((((readBuffer[8] * 256.0) + readBuffer[9]) * 256.0) + readBuffer[10]) / 10000.0;
        if(distance>450.0){
            distance=450.00;
        }
        Serial.printf("distance: %2f cm\n", distance); 

    } else {
        // Print error code
        Serial.print("readI2c failed with error code: ");
        Serial.println(result);
    }
    delay(3000);
}
