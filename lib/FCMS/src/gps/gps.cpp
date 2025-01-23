#include "gps.h"

GPS::GPS(HardwareSerial &serial, bool gpsEcho)
    : gps_(&serial), gpsSerial_(serial), gpsEcho_(gpsEcho), timer_(millis()) {}



void GPS::setup()
{
  gpsSerial_.begin(9600);
  Serial.println("GPS setup");

  changeFrequency();
  delay(100);
  changeBaudrate();
  delay(100);

  gpsSerial_.end();
  gpsSerial_.begin(115200);
  gps_.begin(115200);

  unsigned long start = millis();
  while ((abs(lat) <= EPSYLON || abs(lon) <= EPSYLON) && (millis() - start < GPS_SETUP_TIMEOUT))
  {
    update();
  }
  if (abs(lat) <= EPSYLON && abs(lon) <= EPSYLON)
    Serial.println("Could not configure GPS");
  else
    Serial.println("GPS setup successful");
}


void GPS::update()
{
  unsigned long start = millis();
  while (millis() - start < GET_COORDS_TIMEOUT)
  {
    if (readAndParse())
    {
      lat = gps_.latitude / 100.0;
      lon = gps_.longitude / 100.0;
      return;
    }
  }
}

bool GPS::readLatLon(float &lat_, float &lon_)
{
  char c = gps_.read();
  if (gps_.newNMEAreceived()) {
    if (!gps_.parse(gps_.lastNMEA())) 
      return false;
    else {
      lat_ = gps_.latitude / 100.0;
      lon_ = gps_.longitude / 100.0;
      return true;
    }
  }
}

bool GPS::readAndParse()
{
  char c = gps_.read();
  if (gpsEcho_ && c)
    Serial.print(c);

  if (gps_.newNMEAreceived()){
    return gps_.parse(gps_.lastNMEA());
  }

  return false;
}

void GPS::printStats(){
  Serial.print("Lat: "); Serial.print(gps_.latitude, 4); Serial.print(" ");
  Serial.print("Lon: "); Serial.print(gps_.longitude, 4); Serial.print(" ");
  Serial.print("Alt: "); Serial.print(gps_.altitude, 4); Serial.print(" ");
  Serial.print("Speed: "); Serial.print(gps_.speed, 4); Serial.println();
}

nmea_float_t GPS::getLongitude()
{
  return gps_.longitude / 100;
}

nmea_float_t GPS::getLatitude()
{
  return gps_.latitude / 100;
}

nmea_float_t GPS::getAltitude()
{
  return gps_.altitude;
}


void GPS::sendPacket(byte *packet, byte len)
{
  for (byte i = 0; i < len; i++)
  {
    gpsSerial_.write(packet[i]);
  }
}

void GPS::changeBaudrate()
{
  byte packet115200[] = {
      0xB5, 0x62, 0x06, 0x00,
      0x14, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xD0, 0x08,
      0x00, 0x00, 0x00, 0xC2,
      0x01, 0x00, 0x07, 0x00,
      0x03, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xC0, 0x7E,
  };
  sendPacket(packet115200, sizeof(packet115200));
}

void GPS::changeFrequency()
{
  byte packet[] = {
      0xB5, 0x62, 0x06, 0x08,
      0x06, 0x00, 0xC8, 0x00,
      0x01, 0x00, 0x01, 0x00,
      0xDE, 0x6A,
  };
  sendPacket(packet, sizeof(packet));
}
