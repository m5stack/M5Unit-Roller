#include "unit_roller485.hpp"
#include <Arduino.h>
#include <M5Unified.h>

UnitRoller485 Roller485;     // Create a UnitRoller485 object
HardwareSerial mySerial(1);  // Create a hardware serial port object
#define motor485Id 0x00
uint8_t rgbValues[3];  // Array to store R, G, B values
double speedPID[3];    // Array to   P I D values
void setup() {
    M5.begin();
    // Call the begin function and pass arguments
    Roller485.begin(&mySerial, 115200, SERIAL_8N1, 22, 21, false, 10000UL, 112U);
    // Set the motor mode to speed
    Roller485.setMode(motor485Id, 0x01);
    // Set speed Speed and current
    Roller485.setSpeedMode(motor485Id, 2400, 1200);
    // Set the motor to enable
    Roller485.setOutput(motor485Id, 0x01);
}

void loop() {
    int errorCode = Roller485.setMode(motor485Id, 0x01);
    if (errorCode == 1) {
        // Successful operation
        Serial.println("setMode() successful.");
    } else {
        // Print error code
        Serial.print("setMode() failed with error code: ");
        Serial.println(errorCode);
    }
    delay(1000);

    int32_t temperature = Roller485.getActualTemp(motor485Id);
    Serial.printf("temperature: %d\n", temperature);
    delay(1000);
    int32_t vin = Roller485.getActualVin(motor485Id);
    Serial.printf("vin: %d\n", vin);
    delay(1000);
    int32_t encoder = Roller485.getEncoder(motor485Id);
    Serial.printf("encoder: %d\n", encoder);
    delay(1000);
    int rgbMode = Roller485.getRGBMode(motor485Id);
    Serial.printf("RGBMode: %d\n", rgbMode);
    delay(1000);
    int rgbBrightness = Roller485.getRGBBrightness(motor485Id);
    Serial.printf("rgbBrightness: %d\n", rgbBrightness);
    delay(1000);
    int32_t motorId = Roller485.getMotorId(motor485Id);
    Serial.printf("motorId: %d\n", motorId);
    delay(1000);
    int32_t speed = Roller485.getActualSpeed(motor485Id);
    Serial.printf("speed: %d\n", speed);
    delay(1000);
    int32_t position = Roller485.getActualPosition(motor485Id);
    Serial.printf("position: %d\n", position);
    delay(1000);
    int32_t current = Roller485.getActualCurrent(motor485Id);
    Serial.printf("current: %d\n", current);
    delay(1000);
    int8_t mode = Roller485.getMode(motor485Id);
    Serial.printf("mode: %d\n", mode);
    delay(1000);
    int8_t status = Roller485.getStatus(motor485Id);
    Serial.printf("status: %d\n", status);
    delay(1000);
    int8_t error = Roller485.getError(motor485Id);
    Serial.printf("error: %d\n", error);
    int8_t errorRGB = Roller485.getRGB(motor485Id, rgbValues);
    if (errorRGB != 1) {
        // Print error code
        Serial.print("getRGB failed with error code: ");
        Serial.println(errorRGB);
    } else {
        printf("RGB values: R=%d, G=%d, B=%d\n", rgbValues[0], rgbValues[1], rgbValues[2]);
    }

    int8_t errorSpeedPID = Roller485.getSpeedPID(motor485Id, speedPID);
    if (errorSpeedPID != 1) {
        Serial.print("getSpeedPID failed with error code: ");
        Serial.println(errorSpeedPID);
    } else {
        printf("speedPID values: P=%.8lf, I=%.8lf, D=%.8lf\n", speedPID[0], speedPID[1], speedPID[2]);
    }

    delay(1000);
}
