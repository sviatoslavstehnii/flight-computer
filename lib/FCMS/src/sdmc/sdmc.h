#include <SD.h>
#include "../comms/transceiver.h"

// Secure Digital Memory Card (SD)
class SDMC
{
private:
  const int chipSelect = 17;
  size_t flush_buff = 0;
  const size_t flush_len = 20;
  File LOG;
  Transceiver logger{LOG, 0xCC};

public:
  SDMC() = default;
  ~SDMC() = default;

  SDMC(const SDMC &) = delete;
  SDMC &operator=(const SDMC &) = delete;

  void setup();

  bool write(const char *filename, const char *text);
  bool read(const char *filename);

  void remove(const char *path);

  bool logTelemetry(const Telemetry &telemetry);
  bool writeLOG(const uint8_t *buf, size_t len);
  void closeLOG();
};