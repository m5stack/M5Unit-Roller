#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
UnitRollerI2C RollerI2C;  // Create a UNIT_ROLLERI2C object
uint32_t p, i, d;         // Defines a variable to store the PID value
uint8_t r, g, b;
void setup() {
    M5.begin();
    RollerI2C.begin(&Wire, 0x64, 21, 22, 400000);
}

void loop() {
    
    //current mode
    RollerI2C.setMode(3);
    RollerI2C.setCurrent(120000);
    RollerI2C.setOutput(1);
    printf("current: %d\n", RollerI2C.getCurrent());
    delay(100);
    printf("actualCurrent: %d\n", RollerI2C.getCurrentReadback());
    delay(5000);

    // position mode
    RollerI2C.setOutput(0);
    RollerI2C.setMode(2);
    RollerI2C.setPos(2000000);
    RollerI2C.setPosMaxCurrent(100000);
    RollerI2C.setOutput(1);
    RollerI2C.getPosPID(&p, &i, &d);
    printf("PosPID  P: %3.8f  I: %3.8f  D: %3.8f\n", p / 100000.0, i / 10000000.0, d / 100000.0);
    delay(100);
    printf("pos: %d\n", RollerI2C.getPos());
    delay(100);
    printf("posMaxCurrent: %d\n", RollerI2C.getPosMaxCurrent());
    delay(100);
    printf("actualPos: %d\n", RollerI2C.getPosReadback());
    delay(5000);

    //speed mode
    RollerI2C.setOutput(0);
    RollerI2C.setMode(1);
    RollerI2C.setSpeed(240000);
    RollerI2C.setSpeedMaxCurrent(100000);
    RollerI2C.setOutput(1);
    RollerI2C.getSpeedPID(&p, &i, &d);
    printf("SpeedPID  P: %3.8f  I: %3.8f  D: %3.8f\n", p / 100000.0, i / 10000000.0, d / 100000.0);
    delay(100);
    printf("speed: %d\n", RollerI2C.getSpeed());
    delay(100);
    printf("speedMaxCurrent: %d\n", RollerI2C.getSpeedMaxCurrent());
    delay(100);
    printf("actualSpeed: %d\n", RollerI2C.getSpeedReadback());
    delay(5000);

    // encoder mode
    RollerI2C.setOutput(0);
    RollerI2C.setMode(4);
    RollerI2C.setDialCounter(240000);
    RollerI2C.setOutput(1);
    printf("DialCounter:%d\n", RollerI2C.getDialCounter());
    delay(5000);
    printf("temp:%d\n", RollerI2C.getTemp());
    delay(100);
    printf("Vin:%3.2f\n", RollerI2C.getVin() / 100.0);
    delay(100);
    printf("RGBBrightness:%d\n", RollerI2C.getRGBBrightness());
    delay(1000);
    RollerI2C.setRGBBrightness(100);
    delay(100);
    RollerI2C.setRGBMode(1);
    delay(1000);
    RollerI2C.setRGB(TFT_WHITE);
    delay(1000);
    RollerI2C.setRGB(TFT_BLUE);
    delay(2000);
    RollerI2C.setRGB(TFT_YELLOW);
    delay(2000);
    RollerI2C.setRGB(TFT_RED);
    delay(2000);
    RollerI2C.setRGBMode(0);
    delay(100);
    RollerI2C.setKeySwitchMode(1);
    delay(100);
    printf("I2CAddress:%d\n", RollerI2C.getI2CAddress());
    delay(100);
    printf("485 BPS:%d\n", RollerI2C.getBPS());
    delay(100);
    printf("485 motor id:%d\n", RollerI2C.getMotorID());
    delay(100);
    printf("motor output:%d\n", RollerI2C.getOutputStatus());
    delay(100);
    printf("SysStatus:%d\n", RollerI2C.getSysStatus());
    delay(100);
    printf("ErrorCode:%d\n", RollerI2C.getErrorCode());
    delay(100);
    printf("Button switching mode enable:%d\n", RollerI2C.getKeySwitchMode());
    delay(100);
    RollerI2C.getRGB(&r,&g,&b);
    printf("RGB-R: 0x%02X  RGB-G: 0x%02X  RGB-B: 0x%02X\n", r, g, b);
    delay(5000);
}
