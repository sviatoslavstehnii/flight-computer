#include "barometer/bmp280.h"
#include "barometer/bmp180.h"
#include "imu/imu.h"
#include "flash/flash.h"
#include "gps/gps.h"

// FCMS lib
// Flight Computer Managment System 
enum STATE {
  SAFE=0,
  IDLE=1,
  FLIGHT=2,
  DESCENT=3,
  LANDING=4,
  LANDED=5
};


class FCMS {
  private:
    IMU imu_;
    BMP280 baro_;
    Flash flash_;

    STATE curr_state_;
  public:
    FCMS();
    ~FCMS() = default;

    void setState(STATE state);
    bool nextState();
    STATE getState();

    // TODO: implement below methods

    // updates some variables related to IMU, barometer and gps
    void navigate();
    // writes data to flash memmory
    void commit();
    


};