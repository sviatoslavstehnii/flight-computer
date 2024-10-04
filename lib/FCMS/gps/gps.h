

#include <Adafruit_GPS.h>

class GPS {
  private:
    Adafruit_GPS gps_;
    bool gpsEcho_;
    uint32_t timer_;

  public:
    GPS(HardwareSerial *serial, bool gpsEcho = false);
    void setup();
    bool readAndParse();
    void printStats();

    nmea_float_t getAltitude();
    nmea_float_t getLatitude();
    nmea_float_t getLongitude();

  private:
    void printTime();
    void printDate();
    void printLocation();
    void printOtherStats();
    
};

