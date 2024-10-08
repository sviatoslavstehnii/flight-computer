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
  
  partition();
}

void Flash::partition()
{
  Serial.println("Partition...");
  size_t capacity = flash_.getCapacity();
  
  MEJ_curr_ = 0;
  MEJ_end_ = capacity * 0.25;

  CJ_curr_ = MEJ_end_ + 1;
  CJ_end_ = capacity * 0.5;

  DJ_curr_ = CJ_end_ + 1;
  DJ_end_ = capacity;

}

void Flash::write(uint32_t address, String &data)
{
  flash_.writeStr(address, data);
}

void Flash::write(String &data)
{
  flash_.writeStr(flash_.getAddress(data.length()), data);
}

bool Flash::writeToMEJ(String& data)
{
  size_t l = data.length();
  MEJ_curr_ += l;
  if ( data.length() < 5 ) {
    MEJ_curr_ += l;
  }
  if ( MEJ_curr_ >= MEJ_end_ ) {
    return false;
  }
  write(MEJ_curr_, data);
  MEJ_addresses_.push_back(MEJ_curr_);
  MEJ_curr_ += l;

  return true;
}

bool Flash::writeToCJ(String &data)
{
  size_t l = data.length();
  CJ_curr_ += l;
  if ( data.length() < 5 ) {
    CJ_curr_ += l;
  }
  if ( CJ_curr_ >= CJ_end_ ) {
    return false;
  }
  write(CJ_curr_, data);
  CJ_addresses_.push_back(CJ_curr_);
  CJ_curr_ += l;

  return true;
}

bool Flash::writeToDJ(String &data)
{
  size_t l = data.length();
  DJ_curr_ += l;
  if ( data.length() < 5 ) {
    DJ_curr_ += l;
  }
  if ( DJ_curr_ >= DJ_end_ ) {
    return false;
  }
  write(DJ_curr_, data);
  DJ_addresses_.push_back(DJ_curr_);
  DJ_curr_ += l;

  return true;
}


bool Flash::read(uint32_t address, String& data)
{
  return flash_.readStr(address, data);
}

void Flash::readMEJ(String &data)
{
  for (size_t addr : MEJ_addresses_ ) {
    String temp;

    read(addr, temp);
    data += temp;
    data += "\n";
  }
}

void Flash::readCJ(String &data)
{
  for (size_t addr : CJ_addresses_ ) {
    String temp;

    read(addr, temp);
    data += temp;
    data += "\n";
  }
}

void Flash::readDJ(String &data)
{
  for (size_t addr : DJ_addresses_ ) {
    String temp;

    read(addr, temp);
    data += temp;
    data += "\n";
  }
}

void Flash::clear()
{
  // can last for a couple of seconds/minutes
  flash_.eraseChip();
}
