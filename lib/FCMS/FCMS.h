#include "barometer/bmp280.h"
#include "barometer/bmp180.h"
#include "imu/imu.h"
#include "imu/kalman_filter.h"
#include "flash/flash.h"
#include "gps/gps.h"
#include "sdmc/sdmc.h"
#include "ina/ina.h"

#include <sstream>
#include <iomanip>



// FCMS lib
// Flight Computer Managment System 
enum STATE {
  SAFE=1,
  IDLE=2,
  LAUNCH=3,
  FLIGHT=4,
  NO_POWER=5,
  DESCENT=6,
  PARACHUTE_LANDING=7,
  LANDED=8,
  ABORT=9
};



class FCMS {
  private:
    IMU imu_;
    KalmanFilter kf_;
    BMP280 baro_;
    Flash flash_;
    SDMC sdmc_;

    STATE curr_state_;

    float pitch_ = 0; float roll_ = 0;
  public:
    FCMS(): flash_(10), kf_(0.004) {};
    ~FCMS() = default;

    void setup();

    void setState(STATE state);
    bool nextState();
    STATE getState();    
    
    bool takeoffDetected = imu_.takeoffDetected;
    bool apogeeDetected = baro_.apogeeDetected;
    bool landingDetected = imu_.landingDetected;

    // TODO: implement below methods

    // updates some variables related to IMU, barometer and gps
    void navigate();
    // writes data to flash memmory
    void commit();

    void goToState(STATE state);
    
};