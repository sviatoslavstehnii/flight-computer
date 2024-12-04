#ifndef LORA_H
#define LORA_H

#include <SPI.h>

class LORA {
public:
    LORA(uint8_t csPin, uint8_t resetPin, uint8_t dio0Pin);

    void begin();
    void setFrequency(long frequency);
    void setTxPower(uint8_t level);
    void setSpreadingFactor(uint8_t sf);
    void sendPacket(const uint8_t *data, uint8_t len);
    bool receivePacket(uint8_t *data, uint8_t &len);

private:
    uint8_t _csPin, _resetPin, _dio0Pin;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void resetModule();
};

#endif // LORA_H
