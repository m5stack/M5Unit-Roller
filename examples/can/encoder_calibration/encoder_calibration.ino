#include <Arduino.h>
#include <unit_rollercan.hpp>

constexpr uint8_t kMotorId = 0xA8;
UnitRollerCAN roller;

void setup()
{
    Serial.begin(115200);
    roller.beginCAN(1000000, 16, 17);
    Serial.println("Send 's' to start calibration or 'w' to save a completed calibration.");
}

void loop()
{
    if (Serial.available() > 0) {
        char command = static_cast<char>(Serial.read());
        if (command == 's') {
            Serial.printf("start result=%d\n", roller.startAngleCal(kMotorId));
        } else if (command == 'w') {
            Serial.printf("save result=%d\n", roller.updateAngleCal(kMotorId));
        }
    }

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 500) {
        lastUpdate    = millis();
        bool busy     = false;
        int8_t result = roller.getCalBusyStatus(kMotorId, &busy);
        Serial.printf("busy=%u result=%d\n", busy, result);
    }
}
