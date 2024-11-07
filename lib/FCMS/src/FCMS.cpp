#include "FCMS.h"


void FCMS::setup()
{
  Serial.println("Set up FCMS");
  Wire.begin();
  delay(100);

  imu_.setup();
  delay(200);

  baro_.setup();
  delay(200);

  curr_state_ = SAFE;

  // set capacity and erase chip
  flash_.setup(16777216, true);
  // write first bytes for consistency
  char setup_data[] = "setup";
  if (!flash_.writeToMEJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }
  flash_.clear();

  sdmc_.setup();
  delay(200);
}


STATE FCMS::getState()
{
  return curr_state_;
}

void FCMS::goToState(STATE state)
{
  curr_state_ = state;
}

void FCMS::navigateFilter()
{
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

void FCMS::commitFlash()
{
  std::ostringstream msg;
  msg << "Pitch: " << std::fixed << std::setprecision(2) << pitch_ << ", Roll: " << std::fixed << std::setprecision(2) << roll_;
  std::string msg_str = msg.str();

  char buf[msg_str.size() + 1];
  std::copy(msg_str.begin(), msg_str.end(), buf);
  buf[msg_str.size()] = '\0';
  
  flash_.writeToMEJ(buf, sizeof(buf));

  // char readData[1000] = "";
  // flash_.readDJ(readData, 1000);
  // Serial.print("Data Journal: ");
  // Serial.println(readData);
}




void FCMS::updateData() {
  baro_.update();
  // baro_.printAltitude();

  imu_.update();
  // imu_.printAccelData();
  // imu_.printGyroData();

  delay(100);
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
    }

    if (state != SAFE || state != LANDED) {
      if (millis() - commitFlashTimer > frequency) {
        navigateFilter();
        commitFlash();
        commitFlashTimer = millis();
      }
    }

  switch (state) {
  case SAFE:
    Serial.println("SAFE");

    if (input_user == "2") {
      Serial.print("i got frrom uses input ");
      Serial.println(input_user.c_str());
      goToState(IDLE);
    }
    break;

  case IDLE:
    Serial.println("IDLE");

    if (!dataLogingStarted) {
      Serial.println("Start writing data to flash");
      dataLogingStarted = true;
    }

    if (input_user == "3") {
      Serial.print("i got frrom uses input ");
      Serial.println(input_user.c_str());
      goToState(LAUNCH);
    }

    break;

  case LAUNCH:
    Serial.println("LAUNCH");
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
    }

    if (imu_.getTakeoffDetected()) {
      Serial.println("Takeoff detected!");
      goToState(FLIGHT);
    }
    break;

  case FLIGHT:
    Serial.println("FLIGHT");

    if (imu_.getAccelX() < 1.5f) { //m/s^2
      goToState(NO_POWER);
    }

    if (baro_.getApogeeDetected()) {
      Serial.println("Apogee detected!");
      Serial.print("Max apogee: ");
      Serial.println(baro_.getMaxApogee());
      goToState(DESCENT);
    }

    break;

  case NO_POWER:
    Serial.println("NO POWER");
    if (baro_.getApogeeDetected()) {
      Serial.println("Apogee detected!");
      goToState(DESCENT);
    }

    if (baro_.getAltitude() < 70.0f) {  //m
      goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
    }

    break;

  case DESCENT:
    Serial.println("DESCENT");
    if (baro_.getAltitude() < 70.0f) {  //m
      goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
    }

    // write 2 time faster
    frequency = 500;

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
    }

    break;
  case LANDED:
    Serial.println("LANDED");
    delay(2000);

      // write data to sd card
      if (!dataWrittenToSD) {
        char readData[1000] = "";
        flash_.readMEJ(readData, 1000);
        sdmc_.write("landed.txt", readData);
        dataWrittenToSD = true;
        Serial.println("Data written to SD card");
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
      goToState(LANDED);
    }

    break;
  default:
    break;
  }
}