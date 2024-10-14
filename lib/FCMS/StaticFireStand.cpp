#include "StaticFireStand.h"

void StaticFireStand::setup()
{
  Serial.println("Set up StaticFireStand");
  Wire.begin();
  
  loadcell_.setup();
  delay(100);
  
  // set capacity and erase chip
  flash_.setup(16777216, true);
  // write first bytes for consistency
  char setup_data[] = "setup logging";
  if (!flash_.writeToMEJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }
  Serial.println("StaticFireStand setup complete");
}

void StaticFireStand::setState(SFS_STATE state)
{
  curr_state_ = state;
}

bool StaticFireStand::nextState()
{
  if (curr_state_ < 3) {
    curr_state_ = static_cast<SFS_STATE>(static_cast<int>(curr_state_) + 1);;
    return true;
  }
  return false;
}

SFS_STATE StaticFireStand::getState()
{
  return curr_state_;
}

void StaticFireStand::arm()
{
  pyro_.arm();
}

void StaticFireStand::disarm()
{
  pyro_.disarm();
}

void StaticFireStand::fire()
{
  pyro_.fire();
}

void StaticFireStand::tareLoadCell()
{
  loadcell_.tare();
}

void StaticFireStand::calibrateLoadCell(float known_mass)
{
  loadcell_.calibrate(known_mass);
}

void StaticFireStand::monitor()
{
  uint32_t time_ms = millis();
  // update loadcell
  float weight = loadcell_.getWeight();
  // update pyro
  PyroState pyro_state = pyro_.getState();
  
  // store data
  data_points_[data_index_].time_ms = time_ms;
  data_points_[data_index_].weight = weight;
  data_points_[data_index_].pyro_state = pyro_state;
  data_points_[data_index_].sfs_state = curr_state_;
  ++data_index_;

    // print for debug
    Serial.print("time: ");
    Serial.print(time_ms);
    Serial.print(", ");
    Serial.print("weight: ");
    Serial.print(weight);
    Serial.print(", ");
    Serial.print("pyro state: ");
    Serial.print(pyro_state);
    Serial.print(", ");
    Serial.print("sfs state: ");
    Serial.println(curr_state_);
}

void StaticFireStand::commit()
{
  // write data to flash
  for (int i = 0; i < data_index_; i++) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << data_points_[i].time_ms << "," << data_points_[i].weight << "," << data_points_[i].pyro_state << "," << data_points_[i].sfs_state;
    std::string str = ss.str();
    int str_len = str.length();
    char data[str_len+1] = "";
    strcpy(data, str.c_str());
    if (!flash_.writeToDJ(data, sizeof(data)) ) {
      char error_msg[42] = "ERROR: failed to write data to flash chip";
      Serial.println(error_msg);
      flash_.writeToMEJ(error_msg, 42);
    }
  }
  data_index_ = 0;
}
