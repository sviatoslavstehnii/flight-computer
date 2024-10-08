#include <SPIMemory.h>
#include <vector>

class Flash {
  private:
    SPIFlash flash_;

    size_t MEJ_curr_ = 0;
    size_t MEJ_end_ = 0;
    std::vector<size_t> MEJ_addresses_;

    size_t CJ_curr_ = 0;
    size_t CJ_end_ = 0;
    std::vector<size_t> CJ_addresses_;

    size_t DJ_curr_ = 0;
    size_t DJ_end_ = 0;
    std::vector<size_t> DJ_addresses_;

  public:
    Flash() = default;
    ~Flash() = default;

    Flash(const Flash&) = delete;
    Flash& operator=(const Flash&) = delete;

    void setup(bool erase=false);
    void partition();

    void write(uint32_t address, String& data);
    void write(String& data);
    bool writeToMEJ(String& data);
    bool writeToCJ(String& data);
    bool writeToDJ(String& data);

    bool read(uint32_t address, String& data);
    
    void readMEJ(String& data);
    void readCJ(String& data);
    void readDJ(String& data);

    void clear();


};