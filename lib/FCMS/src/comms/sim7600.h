#ifndef SIM7600_H
#define SIM7600_H

#include "transceiver.h"

class SIM7600 : public Transceiver
{
public:
    SIM7600(HardwareSerial &serial, uint8_t myAddr, size_t tel_queue_size = 10, size_t com_queue_size = 10, size_t resp_queue_size = 10)
        : Transceiver(serial, myAddr, tel_queue_size, com_queue_size, resp_queue_size), serial(serial) {};

    ~SIM7600();
    bool setup();

private:
    HardwareSerial &serial;
};

#endif