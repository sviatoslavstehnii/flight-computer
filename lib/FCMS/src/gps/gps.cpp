#include "GPS.h"

GPS::GPS(HardwareSerial *serial, bool gpsEcho)
  : gps_(&Serial1), gpsEcho_(gpsEcho), timer_(millis()) {}

void GPS::setup() {
  gps_.begin(9600);
  gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  gps_.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);
}

bool GPS::readAndParse() {
  char c = gps_.read();
  if (gpsEcho_ && c) Serial.print(c);

  if (gps_.newNMEAreceived()) {
    return gps_.parse(gps_.lastNMEA());
  }

  return false;
}

void GPS::printStats() {
  if (millis() - timer_ > 2000) {
    timer_ = millis();

    Serial.print("\nTime: ");
    printTime();

    Serial.print("Date: ");
    printDate();

    Serial.print("Fix: ");
    Serial.print((int)gps_.fix);
    Serial.print(" quality: ");
    Serial.println((int)gps_.fixquality);

    if (gps_.fix) {
        printLocation();
        printOtherStats();
    }
  }
}

void GPS::printTime() {
  if (gps_.hour < 10) Serial.print('0');
  Serial.print(gps_.hour, DEC); Serial.print(':');
  if (gps_.minute < 10) Serial.print('0');
  Serial.print(gps_.minute, DEC); Serial.print(':');
  if (gps_.seconds < 10) Serial.print('0');
  Serial.print(gps_.seconds, DEC); Serial.print('.');
  if (gps_.milliseconds < 10) Serial.print("00");
  else if (gps_.milliseconds < 100) Serial.print('0');
  Serial.println(gps_.milliseconds);
}

void GPS::printDate() {
  Serial.print(gps_.day, DEC); Serial.print('/');
  Serial.print(gps_.month, DEC); Serial.print("/20");
  Serial.println(gps_.year, DEC);
}

void GPS::printLocation() {
  Serial.print("Location: ");
  Serial.print(gps_.latitude, 4); Serial.print(gps_.lat);
  Serial.print(", ");
  Serial.print(gps_.longitude, 4); Serial.println(gps_.lon);
}

void GPS::printOtherStats() {
  Serial.print("Speed (knots): "); Serial.println(gps_.speed);
  Serial.print("Angle: "); Serial.println(gps_.angle);
  Serial.print("Altitude: "); Serial.println(gps_.altitude);
  Serial.print("Satellites: "); Serial.println((int)gps_.satellites);
  Serial.print("Antenna status: "); Serial.println((int)gps_.antenna);
}



nmea_float_t GPS::getLongitude()
{
  return gps_.fix ? gps_.longitude : 0.0f;
}

nmea_float_t GPS::getLatitude()
{
  return gps_.fix ? gps_.latitude : 0.0f;
}

nmea_float_t GPS::getAltitude()
{
  return gps_.fix ? gps_.altitude : 0.0f;
}