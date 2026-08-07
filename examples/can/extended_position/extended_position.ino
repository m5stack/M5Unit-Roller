#include <Arduino.h>
#include <unit_rollercan.hpp>

constexpr uint8_t kMotorId = 0xA8;
UnitRollerCAN roller;

void setup()
{
    Serial.begin(115200);
    roller.beginCAN(1000000, 16, 17);
    roller.setOutput(kMotorId, false);
    roller.setMode(kMotorId, ROLLER_MODE_POSITION);
    roller.setPosTurnAngle(kMotorId, 1, 9000, 500);
}

void loop()
{
    int32_t targetTurn    = 0;
    uint16_t targetAngle  = 0;
    uint16_t maxCurrent   = 0;
    int32_t actualTurn    = 0;
    uint16_t actualAngle  = 0;
    int16_t actualCurrent = 0;
    int8_t targetResult   = roller.getPosTurnAngle(kMotorId, &targetTurn, &targetAngle, &maxCurrent);
    int8_t actualResult   = roller.getPosReadbackTurnAngle(kMotorId, &actualTurn, &actualAngle, &actualCurrent);
    Serial.printf("target=%ld+%.2f/%u mA actual=%ld+%.2f/%d mA result=%d/%d\n", static_cast<long>(targetTurn),
                  targetAngle / 100.0f, maxCurrent, static_cast<long>(actualTurn), actualAngle / 100.0f, actualCurrent,
                  targetResult, actualResult);
    delay(1000);
}
