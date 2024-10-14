#ifndef STATICFIRESTAND_H
#define STATICFIRESTAND_H

#include "flash/flash.h"
#include "loadcell/loadcell.h"
#include "ina/ina.h"
#include "pyro/pyro.h"
#include "sdmc/sdmc.h"

#include <sstream>
#include <iomanip>



// StaticFireStand lib
enum SFS_STATE {
  SAFE=0,
  IDLE=1,
  STATIC_FIRE=2,
  BURNOUT=3
};

struct SFS_DATA {
  uint32_t time_ms;
  float weight;
  int pyro_state;
  int sfs_state;
};

class StaticFireStand {
  private:
    Flash flash_;
    Pyro pyro_;
    LoadCell loadcell_;
    SFS_STATE curr_state_;
    SFS_DATA data_points_[250]{};
    int data_index_ = 0;
  public:
    StaticFireStand(): flash_(10), curr_state_(SAFE), loadcell_(), pyro_(-1111) {};
    ~StaticFireStand() = default;

    void setup();

    void setState(SFS_STATE state);
    bool nextState();
    SFS_STATE getState();

    // updates some variables related to IMU, barometer and gps
    void arm();
    void disarm();
    void fire();

    void tareLoadCell();
    void calibrateLoadCell(float known_mass);

    void monitor();
    // writes data to flash memmory
    void commit();

};

#endif