#include "sdmc.h"

void SDMC::setup()
{
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");
}

bool SDMC::write(const char * filename, const char * text)
{
  File f = SD.open(filename, FILE_WRITE);
  if ( f ) {
    f.println(text);
    f.close();
    return true;
  }

  return false;
}

bool SDMC::read(const char * filname)
{
  File f = SD.open("test.txt");
  if (f) {
    Serial.println("test.txt:");

    while (f.available()) {
      Serial.write(f.read());
    }
    f.close();
  }
  return false;
}

void SDMC::remove(const char *path)
{
  SD.remove(path);
}
