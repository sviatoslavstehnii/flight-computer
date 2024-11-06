#include <SD.h>

// Secure Digital Memory Card (SD)
class SDMC {
  private:
    const int chipSelect = 9;

  public:
    SDMC() = default;
    ~SDMC() = default;

    SDMC(const SDMC&) = delete;
    SDMC& operator=(const SDMC&) = delete;

    void setup();

    bool write(const char * filename, const char * text);
    bool read(const char * filename);

    void remove(const char * path);
};