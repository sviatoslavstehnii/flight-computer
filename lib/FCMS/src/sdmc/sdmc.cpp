#include "sdmc.h"

void SDMC::setup()
{
  if (!SD.begin(chipSelect))
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
  }
  else
  {
    Serial.println("SD card initialization done.");
    LOG = SD.open("TELEMETRY.txt", FILE_WRITE);
  }
}

bool SDMC::write(const char *filename, const char *text)
{
  File f = SD.open(filename, FILE_WRITE);
  if (f)
  {
    f.println(text);
    f.close();
    return true;
  }
  f.close();

  return false;
}

bool SDMC::logTelemetry(const Telemetry &telemetry)
{
  if (LOG)
  {
    logger.sendTelemetry(0xEE, telemetry);
    flush_buff += 1;
    if (flush_buff >= flush_len)
    {
      LOG.flush();
      flush_buff = 0;
    }
    return true;
  }

  return false;
}

void SDMC::closeLOG()
{
  if (LOG)
  {
    LOG.flush();
    LOG.close();
  }
}

bool SDMC::read(const char *filename)
{
  File f = SD.open(filename);
  if (f)
  {

    while (f.available())
    {
      Serial.write(f.read());
    }
    f.close();
    return true;
  }
  f.close();
  return false;
}

void SDMC::remove(const char *path)
{
  if (SD.exists(path))
  {
    SD.remove(path);
  }
}
