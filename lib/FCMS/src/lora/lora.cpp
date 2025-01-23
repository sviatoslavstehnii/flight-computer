#include "lora.h"

LORA::LORA(uint8_t csPin, uint8_t resetPin, uint8_t dio0Pin)
    : _csPin(csPin), _resetPin(resetPin), _dio0Pin(dio0Pin) {}

void LORA::begin() {
    pinMode(_csPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_dio0Pin, INPUT);
    digitalWrite(_csPin, HIGH);

    SPI.begin();
    resetModule();
}

void LORA::resetModule() {
    digitalWrite(_resetPin, LOW);
    delay(10);
    digitalWrite(_resetPin, HIGH);
    delay(10);
}

void LORA::setFrequency(long frequency) {
    // Frequency calculation: frf = (frequency / 32e6) * 524288
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    writeRegister(0x06, (frf >> 16) & 0xFF);
    writeRegister(0x07, (frf >> 8) & 0xFF);
    writeRegister(0x08, frf & 0xFF);
}

void LORA::setTxPower(uint8_t level) {
    writeRegister(0x09, (level > 17 ? 17 : level) + 2); // Set PA_BOOST
}

void LORA::setSpreadingFactor(uint8_t sf) {
    uint8_t reg = readRegister(0x1D);
    writeRegister(0x1D, (reg & 0x0F) | ((sf << 4) & 0xF0));
}

void LORA::sendPacket(const uint8_t *data, uint8_t len) {
    writeRegister(0x0D, 0x00); // Set FIFO pointer
    for (uint8_t i = 0; i < len; i++) {
        writeRegister(0x00, data[i]);
    }
    writeRegister(0x22, len); // Payload length
    writeRegister(0x01, 0x83); // Set to TX mode
    while (!(readRegister(0x12) & 0x08)); // Wait for TX done
    writeRegister(0x12, 0x08); // Clear TX done flag
}

bool LORA::receivePacket(uint8_t *data, uint8_t &len) {
    if (!(readRegister(0x12) & 0x40)) return false; // Check RX done
    writeRegister(0x12, 0x40); // Clear RX done flag

    uint8_t size = readRegister(0x13);
    for (uint8_t i = 0; i < size; i++) {
        data[i] = readRegister(0x00);
    }
    len = size;
    return true;
}

void LORA::writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(value);
    digitalWrite(_csPin, HIGH);
}

uint8_t LORA::readRegister(uint8_t reg) {
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F);
    uint8_t value = SPI.transfer(0);
    digitalWrite(_csPin, HIGH);
    return value;
}
