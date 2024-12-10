#include "FCMS.h"


void FCMS::setup()
{
  Serial.println("Set up FCMS");
  Wire.begin();
  sdmc_.setup();

  gps_.setup();
  baro388_.setup();
  baro_.setup();
  imu_.setup();
  imu9dof_.setup();  
  delay(20000);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  delay(140);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
    delay(140);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  delay(140);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(140);
  digitalWrite(BUZZER_PIN, LOW);
  // set capacity and erase chip
  flash_.setup(16777216, true);
  // write first bytes for consistency
  char setup_data[] = "setup";
  if (!flash_.writeToDJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }
  curr_state_ = LAUNCH;
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

  Serial.println("Check health of imu9dof...");
  prev_val = 0;
  while (true) {
    imu9dof_.update();
    float new_val = imu9dof_.getAccelX();
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
  float rollRate, angleRoll, pitchRate, anglePitch = 0.0f;
  //first imu
  imu9dof_.update();
  // rollRate = imu9dof_.getRollRate();
  // angleRoll = imu9dof_.getAngleRoll();

  // pitchRate = imu9dof_.getPitchRate();
  // anglePitch = imu9dof_.getAnglePitch();

  // kf9dof_.updateRoll(rollRate, angleRoll);
  // kf9dof_.updatePitch(pitchRate, anglePitch);
  // sensor_data_.pitch1 = kf9dof_.getAnglePitch();
  // sensor_data_.roll1 = kf9dof_.getAngleRoll();
  // sensor_data_.yaw1 = imu9dof_.getYawRate();

  sensor_data_.pitch1 = imu9dof_.getPitchRate();
  sensor_data_.roll1 = imu9dof_.getRollRate();
  sensor_data_.yaw1 = imu9dof_.getYawRate();
  
  // second imu
  imu_.update();
  rollRate = imu_.getRollRate();
  angleRoll = imu_.getAngleRoll();
  pitchRate = imu_.getPitchRate();
  anglePitch = imu_.getAnglePitch();
  kf_.updateYaw(imu_.getYawRate());
  kf_.updateRoll(rollRate, angleRoll);
  kf_.updatePitch(pitchRate, anglePitch);

  sensor_data_.pitch2 = kf_.getAnglePitch();
  sensor_data_.roll2 = kf_.getAngleRoll();
  sensor_data_.yaw2 = kf_.getAngleYaw();


  // print for debug
  // Serial.print("pitch: ");
  // Serial.print(sensor_data_.pitch1);
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.print(sensor_data_.roll1);
  // Serial.print(", ");
  // Serial.print("yaw: ");
  // Serial.println(sensor_data_.yaw1);
}

void FCMS::estimateAltitude()
{


  baro_.update();
  sensor_data_.alt1 = baro_.getAltitude();

  baro388_.update();
  sensor_data_.alt2 = baro388_.getAltitude();

}

void FCMS::estimateGPS()
{
  if (gps_.readAndParse()){
    Serial.println("GPS data:");
    sensor_data_.lat = gps_.getLatitude();
    sensor_data_.lon = gps_.getLongitude();
  }
}

void FCMS::commitFlash()
{

  std::ostringstream data_msg;
  data_msg << "Pitch:" << std::fixed << std::setprecision(2) << sensor_data_.pitch1 <<
   ",Roll: " << std::fixed << std::setprecision(2) << sensor_data_.roll1 
  << ",Yaw: " << std::fixed << std::setprecision(2) << sensor_data_.yaw1
  << ",Alt:" << std::fixed << std::setprecision(2) << sensor_data_.alt1
  << ",Lat:" << std::fixed << std::setprecision(2) << sensor_data_.lat
  << ",Lon:" << std::fixed << std::setprecision(2) << sensor_data_.lon << ";"<< getState();
  std::string data_msg_str = data_msg.str();

  char data_buf[data_msg_str.size() + 1];
  std::copy(data_msg_str.begin(), data_msg_str.end(), data_buf);
  data_buf[data_msg_str.size()] = '\0';
  
  if (!flash_.writeToDJ(data_buf, sizeof(data_buf))) {
    Serial.println("error");
  }

  std::ostringstream data_msg2;
  data_msg2 << "|2|Pitch:" << std::fixed << std::setprecision(2) << sensor_data_.pitch2
          << ",Roll: " << std::fixed << std::setprecision(2) << sensor_data_.roll2
          << ",Yaw: " << std::fixed << std::setprecision(2) << sensor_data_.yaw2
          << ",Alt:" << std::fixed << std::setprecision(2) << sensor_data_.alt2 
          << ";" << getState();

  std::string data_msg_str2 = data_msg2.str();

  char data_buf2[data_msg_str2.size() + 1];
  std::copy(data_msg_str2.begin(), data_msg_str2.end(), data_buf2);
  data_buf2[data_msg_str2.size()] = '\0';

  if (!flash_.writeToDJ(data_buf2, sizeof(data_buf2))) {
      Serial.println("Error writing data");
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
      break;
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
      break;
    }
    bytes_read += strlen(read_data);
  }



}

void FCMS::step()
{
  STATE state = getState();


  if (state != SAFE && state != LANDED) {
    estimateAttitude();
    if(millis() - estimateAltitudeMillis >= estimateAltitudeInterval) {
      estimateAltitudeMillis = millis();
      estimateAltitude();
    }
    if ((millis() - commitMillis >= commitInterval) && dataLogingStarted) {
      commitMillis = millis();
      commitFlash();
      Serial.print("{\"roll\":");
      Serial.print(sensor_data_.roll1, 2);
      Serial.print(",\"pitch\":");
      Serial.print(sensor_data_.pitch1, 2);
      Serial.print(",\"yaw\":");
      Serial.print(sensor_data_.yaw1, 2);
      Serial.print(",\"lat\":");
      Serial.print(sensor_data_.lat, 6);
      Serial.print(",\"lon\":");
      Serial.print(sensor_data_.lon, 6);
      Serial.print(",\"alt\":");
      Serial.print(sensor_data_.alt1, 2);
      Serial.print(",\"state\":");
      Serial.print(state);
      Serial.println("}");
    }

    if (millis() - estimateGPSMillis >= estimateGPSInterval) {
      estimateGPSMillis = millis();
      estimateGPS();
    }

    // if (millis() - buzzerMillis >= 1500) {
    //   buzzerMillis = millis();
    //   digitalWrite(BUZZER_PIN, LOW);
    // } 
  }
  updateState();

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
    Serial.println("LAUNCH");
    if (!dataLogingStarted) {
      Serial.println("Start writing data to flash");
      dataLogingStarted = true;
    }
    if (firstlaunch) {
      launchAbortTime = millis();
      firstlaunch = false;
    }

    // turn pyro on
    if (!pyroOn) {
      Serial.println("Turn pyro on");
      pyroOn = true;
    }
        

    if (millis() - launchAbortTime > 500000) {
      goToState(ABORT);
      major_events_q_.push({"ABORT", millis()});
    }

    if (imu_.getTakeoffDetected()) {
      Serial.println("Takeoff detected!");
      goToState(FLIGHT);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);

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
      digitalWrite(BUZZER_PIN, HIGH);

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
      ++land;
    } else {
      land = 0;
    }
    if (land > 20) {
    Serial.println("Landing detected!");
      goToState(LANDED);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);

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
    Serial.println("ABORT");

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
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);


      goToState(LANDED);
      major_events_q_.push({"LANDED", millis()});
    }

    break;
  default:
    break;
  }
}
