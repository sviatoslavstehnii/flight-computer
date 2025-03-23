#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <SD.h>
#include "../comms/transceiver.h"

class Blackbox
{
private:
  const int chipSelect = 17;
  size_t flush_buff = 0;
  const size_t flush_len = 20;
  File LOG;
  Transceiver logger{LOG, 0xCC};

public:
  Blackbox() = default;
  ~Blackbox() = default;

  Blackbox(const Blackbox &) = delete;
  Blackbox &operator=(const Blackbox &) = delete;

  bool setup();

  bool write(const char *filename, const char *text);
  bool read(const char *filename);

  void remove(const char *path);

  bool logTelemetry(const Telemetry &telemetry);
  bool writeLOG(const uint8_t *buf, size_t len);
  void closeLOG();
};

#endif