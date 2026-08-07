#include <Arduino.h>
#include <unit_rolleri2c.hpp>

UnitRollerI2C roller;

void setup()
{
    Serial.begin(115200);
    if (!roller.begin(&Wire, I2C_ADDR, 21, 22, 400000)) {
        Serial.println("Roller I2C not found");
        return;
    }

    roller.setOutput(0);
    roller.setMode(ROLLER_MODE_POSITION);
    roller.setPosTurn(1);
    roller.setPosAngle(9000);
}

void loop()
{
    Serial.printf("target=%ld+%.2f actual=%ld+%.2f\n", static_cast<long>(roller.getPosTurn()),
                  roller.getPosAngle() / 100.0f, static_cast<long>(roller.getPosReadbackTurn()),
                  roller.getPosReadbackAngle() / 100.0f);
    delay(1000);
}
