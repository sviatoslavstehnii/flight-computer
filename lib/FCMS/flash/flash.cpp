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

bool Flash::writeToMEJ(char *buf, size_t len)
{
  if ( len >= 100 ) {
    Serial.println("Buffer should be shorter than 100 chars");
    return false;
  }
  // add \n
  size_t new_l = len + 1;
  char new_buf[new_l];
  memcpy(new_buf, buf, len);
  new_buf[len-1] = '\n';
  new_buf[len] = '\0';


  if ( MEJ_curr_ >= MEJ_end_ ) {
    return false;
  }
  write(MEJ_curr_, new_buf, new_l);
  MEJ_curr_ += new_l;
  return true;
}

bool Flash::writeToCJ(char *buf, size_t len)
{
  if ( len >= 100 ) {
    Serial.println("Buffer should be shorter than 100 chars");
    return false;
  }
  // add \n
  size_t new_l = len + 1;
  char new_buf[new_l];
  memcpy(new_buf, buf, len);
  new_buf[len-1] = '\n';
  new_buf[len] = '\0';


  if ( CJ_curr_ >= CJ_end_ ) {
    return false;
  }
  write(CJ_curr_, new_buf, new_l);
  CJ_curr_ += new_l;
  return true;
}

bool Flash::writeToDJ(char *buf, size_t len)
{
  if ( len >= 100 ) {
    Serial.println("Buffer should be shorter than 100 chars");
    return false;
  }
  // add \n
  size_t new_l = len + 1;
  char new_buf[new_l];
  memcpy(new_buf, buf, len);
  new_buf[len-1] = '\n';
  new_buf[len] = '\0';


  if ( DJ_curr_ >= DJ_end_ ) {
    return false;
  }
  write(DJ_curr_, new_buf, new_l);
  DJ_curr_ += len;
  return new_l;
}

void Flash::read(uint32_t addr, void *buf, uint16_t len)
{
  flash_.readBytes(addr, buf, len);
}

void Flash::readMEJ(void *buf, uint16_t len)
{
  char mej_data[len] = "";
  if ( len < 100 ) {
    Serial.println("You should read at least 100 bytes.");
    return;
  }
  size_t offset = 0;
  size_t chunk_sz = 100;
  while (true) {
    char temp_buf[chunk_sz];
    read(MEJ_start_+offset, temp_buf, chunk_sz);
    size_t temp_sz = strlen(temp_buf);

    if ( temp_sz == 0 || temp_buf[temp_sz-1] != '\n') {
      break;
    }
    
    if ( strlen(mej_data)+temp_sz >= len ) { 
      break;
    }

    memcpy(mej_data+strlen(mej_data), temp_buf, temp_sz);

    offset += temp_sz+1;
  }

  memcpy(buf, mej_data, strlen(mej_data));
}

void Flash::readCJ(void *buf, uint16_t len)
{

  char cj_data[len] = "";
  if ( len < 100 ) {
    Serial.println("You should read at least 100 bytes.");
    return;
  }
  size_t offset = 0;
  size_t chunk_sz = 100;
  while (true) {
    char temp_buf[chunk_sz];
    read(CJ_start_+offset, temp_buf, chunk_sz);
    size_t temp_sz = strlen(temp_buf);

    if ( temp_sz == 0 || temp_buf[temp_sz-1] != '\n') {
      break;
    }
    
    if ( strlen(cj_data)+temp_sz >= len ) { 
      break;
    }

    memcpy(cj_data+strlen(cj_data), temp_buf, temp_sz);

    offset += temp_sz+1;
  }

  memcpy(buf, cj_data, strlen(cj_data));
}

void Flash::readDJ(void *buf, uint16_t len)
{

  char dj_data[len] = "";
  if ( len < 100 ) {
    Serial.println("You should read at least 100 bytes.");
    return;
  }
  size_t offset = 0;
  size_t chunk_sz = 100;
  while (true) {
    char temp_buf[chunk_sz];
    read(DJ_start_+offset, temp_buf, chunk_sz);
    size_t temp_sz = strlen(temp_buf);

    if ( temp_sz == 0 || temp_buf[temp_sz-1] != '\n') {
      break;
    }
    
    if ( strlen(dj_data)+temp_sz >= len ) { 
      break;
    }

    memcpy(dj_data+strlen(dj_data), temp_buf, temp_sz);

    offset += temp_sz+1;
  }

  memcpy(buf, dj_data, strlen(dj_data));
}

void Flash::clear()
{
  // can last for a couple of seconds
  flash_.chipErase();
  Serial.println("Erased flash chip");
}
