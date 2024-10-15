
#ifndef FLASH_H
#define FLASH_H


#include <SPI.h>
#include <SPIFlash.h>
#include <Wire.h>

class Flash {
  private:
    SPIFlash flash_;
    
    size_t MEJ_start_ = 0;
    size_t MEJ_curr_ = 0;
    size_t MEJ_end_ = 0;

    size_t CJ_start_ = 0;
    size_t CJ_curr_ = 0;
    size_t CJ_end_ = 0;

    size_t DJ_start_ = 0;
    size_t DJ_curr_ = 0;
    size_t DJ_end_ = 0;

    void partition(size_t capacity);
    
    void write(uint32_t addr, const void *buf, uint16_t len);
    bool writeToPartition(char *buf, size_t len, size_t &curr, size_t end);

    void read(uint32_t addr, void *buf, uint16_t len);
    bool readFromPartition(char *buf, size_t len, size_t start);
    
  public:
    Flash(size_t chipSelect): flash_(chipSelect) {};
    ~Flash() = default;

    Flash(const Flash&) = delete;
    Flash& operator=(const Flash&) = delete;

    void setup(size_t capacity, bool erase);

    bool writeToMEJ(char *buf, size_t len);
    bool writeToCJ(char *buf, size_t len);
    bool writeToDJ(char *buf, size_t len);

    // read at least 100 bytes
    void readMEJ(char *buf, size_t len, size_t bytes_skip = 0);
    void readCJ(char *buf, size_t len, size_t bytes_skip = 0);
    void readDJ(char *buf, size_t len, size_t bytes_skip = 0);

    void clear();


};

#endif // FLASH_H