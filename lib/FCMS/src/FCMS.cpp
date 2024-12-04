#include "FCMS.h"


void FCMS::setup()
{
  Serial.println("Set up FCMS");
  Wire.begin();
  // set capacity and erase chip
  flash_.setup(16777216, true);
  // write first bytes for consistency
  char setup_data[] = "setup";
  if (!flash_.writeToDJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }

  baro388_.setup();
  baro_.setup();
  imu_.setup();
  gps_.setup();


  // sdmc_.setup();

  curr_state_ = SAFE;
}

void FCMS::checkHealth()
{

  Serial.println("Check health of gps...");
  while (true) {
    if (gps_.readAndParse()) {
      gps_.printStats();
      auto val = gps_.getLatitude();
      if (val != 0.0f) {
        Serial.println("Healthy");
        break;
      }
    }
  }

  Serial.println("Check health of imu...");
  float prev_val = 0;
  while (true) {
    imu_.update();
    float new_val = imu_.getAccelX();
    if (new_val != prev_val && prev_val != 0) {
      Serial.println("Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  Serial.println("Check health of barometer388...");
  prev_val = 0;
  while (true) {
    baro388_.update();
    float new_val = baro388_.getAltitude();
    if (new_val != prev_val && prev_val != 0) {
      Serial.println("Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  Serial.println("Check health of barometer280...");
  prev_val = 0;
  while (true) {
    baro_.update();
    float new_val = baro_.getAltitude();
    if (new_val != prev_val && prev_val != 0) {
      Serial.println("Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  

  Serial.println("Sensors are ready to work!");
}

void FCMS::step()
{
  STATE state = getState();


  if (state != SAFE && state != LANDED) {

    if(millis() - estimateAttitudeMillis >= estimateAttitudeInterval) {
      estimateAttitudeMillis = millis();
      estimateAttitude();
    }
    if(millis() - estimateAltitudeMillis >= estimateAltitudeInterval) {
      estimateAltitudeMillis = millis();
      estimateAltitude();
    }
    if ((millis() - commitMillis >= commitInterval) && dataLogingStarted) {
      commitMillis = millis();
      commitFlash();
    }
    if (millis() - monitorMillis >= monitorInterval) {
      monitorMillis = millis();
      Serial.println("check health");
      monitorHealth();
    }
  }
  updateState();
}

STATE FCMS::getState()
{
  return curr_state_;
}

void FCMS::goToState(STATE state)
{
  curr_state_ = state;
}

void FCMS::estimateAttitude()
{
  imu_.update();

  kf_.updateRoll(imu_.getRollRate(), imu_.getAngleRoll());
  kf_.updatePitch(imu_.getPitchRate(), imu_.getAnglePitch());
  kf_.updateYaw(imu_.getYawRate());

  pitch_ = kf_.getAnglePitch();
  roll_ = kf_.getAngleRoll();
  yaw_ = kf_.getAngleYaw();


  // print for debug
  Serial.print("pitch: ");
  Serial.print(pitch_);
  Serial.print(", ");
  Serial.print("roll: ");
  Serial.print(roll_);
  Serial.print(", ");
  Serial.print("yaw: ");
  Serial.println(yaw_);
}

void FCMS::estimateAltitude()
{


  baro_.update();
  altitude_ = baro_.getAltitude();

}

void FCMS::commitFlash()
{

  std::ostringstream data_msg;
  data_msg << "Pitch:" << std::fixed << std::setprecision(2) << pitch_ << ",Roll: " << std::fixed << std::setprecision(2) << roll_ 
    << ",Yaw: " << std::fixed << std::setprecision(2) << yaw_ << ",Alt:" << std::fixed << std::setprecision(2) << altitude_ << ";"<< getState();
  std::string data_msg_str = data_msg.str();

  char data_buf[data_msg_str.size() + 1];
  std::copy(data_msg_str.begin(), data_msg_str.end(), data_buf);
  data_buf[data_msg_str.size()] = '\0';
  
  if (!flash_.writeToDJ(data_buf, sizeof(data_buf))) {
    Serial.println("error");
  }

  while ( !major_events_q_.empty()) {
    std::pair<char *, uint32_t> event = major_events_q_.front();
    major_events_q_.pop();
    
    std::ostringstream event_msg;
    event_msg << event.first << "; " << event.second;
    std::string event_msg_str = event_msg.str();

    char event_buf[event_msg_str.size() + 1];
    std::copy(event_msg_str.begin(), event_msg_str.end(), event_buf);
    event_buf[event_msg_str.size()] = '\0';
    
    flash_.writeToMEJ(event_buf, sizeof(event_buf));
    Serial.println(event_buf);
  }

}

void FCMS::commitSDMC()
{
  sdmc_.remove("dj.txt");
  size_t bytes_read = 0;
  while (true) {
    char read_data[1000] = "";
    flash_.readDJ(read_data, 1000, bytes_read);
    if ( strlen(read_data) == 0 ) {
      break;
    }
    if (!sdmc_.write("dj.txt", read_data)) {
      Serial.println("error while writing to file");
    }
    bytes_read += strlen(read_data);
  }


  sdmc_.remove("mej.txt");
  bytes_read = 0;
  while (true) {
    char read_data[1000] = "";
    flash_.readMEJ(read_data, 1000, bytes_read);
    if ( strlen(read_data) == 0 ) {
      break;
    }
    if (!sdmc_.write("mej.txt", read_data)) {
      Serial.println("error while writing to file");
    }
    bytes_read += strlen(read_data);
  }



}

void FCMS::monitorHealth()
{
  Wire.beginTransmission(0x68);
  if (!Wire.endTransmission() == 0) {
    major_events_q_.push({"IMU_FAILURE", millis()});
  }

  Wire.beginTransmission(0x76);
  if (!Wire.endTransmission() == 0) {
    major_events_q_.push({"BAROMETER_FAILURE", millis()});
  }
}

void FCMS::updateState() {
  STATE state = getState();
  std::string input_user;
  while (Serial.available() > 0) {
    char c = Serial.read();
    input_user += c;
  }

  if (input_user == "0") {
    Serial.print("i got frrom uses input ");
    Serial.println(input_user.c_str());
    goToState(ABORT);
    major_events_q_.push({"ABORT", millis()});
  }

  switch (state) {
  case SAFE:
    // Serial.println("SAFE");

    if (input_user == "2") {
      Serial.print("i got frrom uses input ");
      Serial.println(input_user.c_str());
      goToState(IDLE);
      major_events_q_.push({"IDLE", millis()});
    }
    break;

  case IDLE:
    // Serial.println("IDLE");

    if (!dataLogingStarted) {
      Serial.println("Start writing data to flash");
      dataLogingStarted = true;
    }

    if (input_user == "3") {
      Serial.print("i got frrom uses input ");
      Serial.println(input_user.c_str());
      goToState(LAUNCH);
      major_events_q_.push({"LAUNCH", millis()});
    }

    break;

  case LAUNCH:
    // Serial.println("LAUNCH");
    if (firstlaunch) {
      launchAbortTime = millis();
      firstlaunch = false;
    }

    // turn pyro on
    if (!pyroOn) {
      Serial.println("Turn pyro on");
      pyroOn = true;
    }
        

    if (millis() - launchAbortTime > 5000) {
      goToState(ABORT);
      major_events_q_.push({"ABORT", millis()});
    }

    if (imu_.getTakeoffDetected()) {
      Serial.println("Takeoff detected!");
      goToState(FLIGHT);
      major_events_q_.push({"FLIGHT", millis()});
    }
    break;

  case FLIGHT:
    Serial.println("FLIGHT");

    if (imu_.getAccelX() < 1.5f) { //m/s^2
      goToState(NO_POWER);
      major_events_q_.push({"NO_POWER", millis()});
    }

    if (baro_.getApogeeDetected()) {
      Serial.println("Apogee detected!");
      Serial.print("Max apogee: ");
      Serial.println(baro_.getMaxApogee());
      goToState(DESCENT);
      major_events_q_.push({"DESCENT", millis()});
    }

    break;

  case NO_POWER:
    Serial.println("NO POWER");
    if (baro_.getApogeeDetected()) {
      Serial.println("Apogee detected!");
      goToState(DESCENT);
      major_events_q_.push({"DESCENT", millis()});
    }

    if (baro_.getAltitude() < 70.0f) {  //m
      goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
      major_events_q_.push({"PARACHUTE_LANDING", millis()});
    }

    break;

  case DESCENT:
    Serial.println("DESCENT");
    if (baro_.getAltitude() < 70.0f) {  //m
      goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
      major_events_q_.push({"PARACHUTE_LANDING", millis()});
    }

    // write 2 time faster
    commitInterval /= 2;

    break;

  case PARACHUTE_LANDING:
    Serial.println("PARACHUTE LANDING");
    // deploy parachute
    if (!parachuteDeployed) {
      Serial.println("Deploy parachute");
      parachuteDeployed = true;
    }


    // 1 version detect landing
    // if (millis() - landingDetectTime < 15000) {
    //   goToState(LANDED);
    // }

    // 2 version detect landing
    delay(300);
    imu_.detectLanding();
    if (imu_.getLandingDetected()) {
      Serial.println("Landing detected!");
      goToState(LANDED);
      major_events_q_.push({"LANDED", millis()});
    }

    break;
  case LANDED:
    Serial.println("LANDED");

      // write data to sd card
      if (!dataWrittenToSD) {
        delay(2000);
        commitFlash();
        commitSDMC();
        dataWrittenToSD = true;
      }
    break;
  case ABORT:
    // Serial.println("ABORT");

    if (firstAbortLoop)
    {
      // turn pyro off
      Serial.println("Turn pyro off");

      // deploy parachute
      Serial.println("Deploy parachute");
    
      parachuteDeployed = true;
      abortLoopTime = millis();
      firstAbortLoop = false;
    }

    if (millis() - abortLoopTime > 15000)
    {
      goToState(LANDED);
      major_events_q_.push({"LANDED", millis()});
    }

    break;
  default:
    break;
  }
}
