#include<SPIMemory.h>


class Flash {
  private:
    SPIFlash flash_;

  public:
    Flash() = default;
    ~Flash() = default;

    Flash(const Flash&) = delete;
    Flash& operator=(const Flash&) = delete;

    void setup(bool erase=false);

    void write(uint32_t address, String& data);
    void write(String& data);

    bool read(uint32_t address, String& data);

    void clear();


};