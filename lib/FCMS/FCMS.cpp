#include "FCMS.h"

void FCMS::setup()
{
  Serial.println("Set up FCMS");
  Wire.begin();

  imu_.setup();
  delay(100);

  // set capacity and erase chip
  flash_.setup(16777216, true);
  // write first bytes for consistency
  char setup_data[] = "setup";
  if (!flash_.writeToMEJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }
}

void FCMS::setState(STATE state)
{
  curr_state_ = state;
}

bool FCMS::nextState()
{
  if (curr_state_ < 5) {
    curr_state_ = static_cast<STATE>(static_cast<int>(curr_state_) + 1);;
    return true;
  }
  return false;
}

STATE FCMS::getState()
{
  return curr_state_;
}

void FCMS::navigate()
{
  imu_.update();

  kf_.updateRoll(imu_.getRollRate(), imu_.getAngleRoll());
  kf_.updatePitch(imu_.getPitchRate(), imu_.getAnglePitch());

  pitch_ = kf_.getAnglePitch();
  roll_ = kf_.getAngleRoll();

  // print for debug
  // Serial.print("pitch: ");
  // Serial.print(pitch_);
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.println(roll_);
}

void FCMS::commit()
{

  char readData[1000] = "";

  std::ostringstream msg;
  msg << "Pitch: " << std::fixed << std::setprecision(2) << pitch_ << ", Roll: " << std::fixed << std::setprecision(2) << roll_;
  std::string msg_str = msg.str();

  char buf[msg_str.size() + 1];
  std::copy(msg_str.begin(), msg_str.end(), buf);
  buf[msg_str.size()] = '\0';
  
  flash_.writeToDJ(buf, sizeof(buf));

  
  flash_.readDJ(readData, 1000);
  Serial.print("Data Journal: ");
  Serial.println(readData);
}
