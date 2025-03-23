#include "sim7600.h"

SIM7600::~SIM7600()
{
    serial.println("AT+CIPCLOSE=0");
    serial.end();

    Serial.println("4G communication closed.");
}

bool SIM7600::setup()
{
    Serial.println("Configuring 4G communication...");
    serial.begin(115200);
    delay(200);
    // Wait for the module to respond
    uint32_t init_time = millis() + 10000;
    bool success = false;
    while (init_time > millis())
    {
        if (serial.available() > 0 && serial.find("RDY"))
        {
            success = true;
            Serial.println("SIM7600 Ready");
            break;
        }
        delay(100);
    }
    if (!success)
    {
        Serial.println("Could not initialize SIM7600!");
        Serial.println("Maybe SIM7600 is already running.");
        return false;
    }
    serial.println("AT+CIPMODE=1");
    delay(50);
    serial.println("AT+NETOPEN");
    delay(50);
    serial.println("AT+CIPOPEN=0,\"UDP\",\"13.48.57.240\",14550,14500");
    delay(200);
    // Clear the buffer
    uint32_t clear_time = millis() + 500;
    while (serial.available() > 0 && clear_time > millis())
    {
        Serial.write(serial.read());
    }
    Serial.println();
    return true;
}