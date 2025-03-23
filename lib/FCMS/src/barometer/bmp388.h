#include <BMP388_DEV.h>

#include <cfloat>

#include "barometer.h"

class BMP388 : Barometer
{
private:
  BMP388_DEV bmp_;
  float altitude_ = 0.0;
  float altitudeCalibration_ = 0.0;
  float maxapogee = -FLT_MAX;
  const float minDropThreshold = 2.5;
  const float minApogeeAlt = 25;

  bool apogeeDetected = false;
  bool climbing = true;

  void detectApogee();

#ifdef HITL_MODE
  bool hitl = false;
  uint32_t start_time = 0;
  uint32_t time_now = 0;
  uint32_t climb_duration = 11;
  uint32_t decline_duration = 30;
  float max_alt = 650;
  float last_alt = 0;
#endif

public:
  BMP388() = default;
  ~BMP388() = default;

  BMP388(const BMP388 &) = delete;
  BMP388 &operator=(const BMP388 &) = delete;

  bool setup() override;
  bool calibrate();

  void update();

  void printAltitude();
  float getAltitude() override;

  bool getApogeeDetected() { return apogeeDetected; }
  float getMaxApogee() { return maxapogee; }

  void resetApogee()
  {
    apogeeDetected = false;
    maxapogee = 0;
  }

#ifdef HITL_MODE
  void start_hitl()
  {
    hitl = true;
    start_time = 0;
  }
  void stop_hitl()
  {
    hitl = false;
    climbing = true;
  }
#endif
};