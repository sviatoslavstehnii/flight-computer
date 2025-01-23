#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define GET_COORDS_TIMEOUT 2000
#define GPS_SETUP_TIMEOUT 5000
#define EPSYLON 0.0001


class GPS {
  private:
    Adafruit_GPS gps_;
    HardwareSerial &gpsSerial_;
    bool gpsEcho_;
    uint32_t timer_;
    

  public:
    float lat=0.0;
    float lon=0.0;
    GPS(HardwareSerial &serial=Serial1, bool gpsEcho = false);
    void setup();
    bool readAndParse();
    void printStats();
    void update();
    bool readLatLon(float& lat, float& lon);

    nmea_float_t getAltitude();
    nmea_float_t getLatitude();
    nmea_float_t getLongitude();

  private:
    // setup the baudrate and update speed
    void sendPacket(byte *packet, byte len);
    void changeBaudrate();
    void changeFrequency();

    
};

