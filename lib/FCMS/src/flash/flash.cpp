#include "flash.h"

void Flash::setup(size_t capacity, bool erase=false)
{
  if (flash_.initialize()) {
    Serial.println("Flash chip initialized successfully!");
  } else {
    Serial.println("Failed to initialize flash chip.");
  }

  if (erase) {
    clear();
  }
  
  partition(capacity);
}

void Flash::partition(size_t capacity)
{
  Serial.println("Partition...");
  
  MEJ_start_ = 0;
  MEJ_curr_ = 0;
  MEJ_end_ = capacity * 0.25;

  CJ_start_ = MEJ_end_ + 1;
  CJ_curr_ = CJ_start_;
  CJ_end_ = capacity * 0.5;

  DJ_start_ = CJ_end_ + 1;
  DJ_curr_ = DJ_start_;
  DJ_end_ = capacity;

}


void Flash::write(uint32_t addr, const void *buf, uint16_t len)
{
  flash_.writeBytes(addr, buf, len);
}

bool Flash::writeToPartition(char *buf, size_t len, size_t &curr, size_t end)
{
  if (len >= 100) {
    Serial.println("Buffer should be shorter than 100 chars");
    return false;
  }

  // Add newline char
  size_t new_l = len + 1;
  char new_buf[new_l];
  memcpy(new_buf, buf, len);
  new_buf[len-1] = '\n';  
  new_buf[len] = '\0'; 

  if (curr >= end) {
    Serial.println("No mem left");
    return false;
  }


  write(curr, new_buf, new_l);
  curr += new_l; 
  return true;
}

bool Flash::writeToMEJ(char *buf, size_t len)
{
  return writeToPartition(buf, len, MEJ_curr_, MEJ_end_);
}

bool Flash::writeToCJ(char *buf, size_t len)
{
  return writeToPartition(buf, len, CJ_curr_, CJ_end_);
}

bool Flash::writeToDJ(char *buf, size_t len)
{
  return writeToPartition(buf, len, DJ_curr_, DJ_end_);
}

void Flash::read(uint32_t addr, void *buf, uint16_t len)
{
  flash_.readBytes(addr, buf, len);
}

bool Flash::readFromPartition(char *buf, size_t len, size_t start)
{
  char data[len] = "";
  if (len < 100) {
    Serial.println("You should read at least 100 bytes.");
    return false;
  }

  size_t offset = 0;
  size_t chunk_sz = 100;
  int time = millis();

  while (millis() - time < FLASH_READ_TIMEOUT) {
    char temp_buf[chunk_sz];
    read(start + offset, temp_buf, chunk_sz);
    size_t temp_sz = strlen(temp_buf);

    if (temp_sz == 0 || temp_buf[temp_sz - 1] != '\n') {
      break;
    }

    if (strlen(data) + temp_sz >= len) {
      break;
    }

    memcpy(data + strlen(data), temp_buf, temp_sz);

    offset += temp_sz + 1;
  }

  memcpy(buf, data, strlen(data));
  return true;
}

void Flash::readMEJ(char *buf, size_t len, size_t bytes_skip)
{
  readFromPartition(buf, len, MEJ_start_+bytes_skip);
}

void Flash::readCJ(char *buf, size_t len, size_t bytes_skip)
{
  readFromPartition(buf, len, CJ_start_+bytes_skip);
}

void Flash::readDJ(char *buf, size_t len, size_t bytes_skip)
{
  readFromPartition(buf, len, DJ_start_+bytes_skip);
}

void Flash::clear()
{
  // can last for a couple of seconds
  flash_.chipErase();
  Serial.println("Erased flash chip");
}
