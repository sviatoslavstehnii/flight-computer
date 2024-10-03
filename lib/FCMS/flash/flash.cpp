#include "flash.h"

void Flash::setup(bool erase)
{
  Serial.println("Setup W25Q...");
  flash_.setClock(12000000);

  bool ok = flash_.begin();
  if (!ok) {
    Serial.println("Problem while beginning working with flash mem.");
    return;
  }
  if (erase)
    clear();
}

void Flash::write(uint32_t address, String &data)
{
  flash_.writeStr(address, data);
}

void Flash::write(String &data)
{
  flash_.writeStr(flash_.getAddress(data.length()), data);
}

bool Flash::read(uint32_t address, String& data)
{
  return flash_.readStr(address, data);
}

void Flash::clear()
{
  flash_.eraseChip();
}
