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
  if (!flash_.writeToDJ(setup_data, sizeof(setup_data)) ) {
    Serial.println("ERROR: failed to write setup data to flash chip");
  }

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

void FCMS::estimate()
{
  imu_.update();

  kf_.updateRoll(imu_.getRollRate(), imu_.getAngleRoll());
  kf_.updatePitch(imu_.getPitchRate(), imu_.getAnglePitch());
  kf_.updateYaw(imu_.getYawRate());

  pitch_ = kf_.getAnglePitch();
  roll_ = kf_.getAngleRoll();
  yaw_ = kf_.getAngleYaw();

  baro_.update();
  altitude_ = baro_.getAltitude();

  // print for debug
  // Serial.print("pitch: ");
  // Serial.print(pitch_);
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.print(roll_);
  // Serial.print(", ");
  // Serial.print("yaw: ");
  // Serial.println(yaw_);
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

  if (state != SAFE && state != LANDED) {

    if(millis() - estimateMillis >= estimateInterval) {
      estimateMillis = millis();
      estimate();
    }
    if ((millis() - commitMillis >= commitInterval) && dataLogingStarted) {
      commitMillis = millis();
      commitFlash();
    }
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



// haversine formula
float haversine(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371.0f; // Radius of the earth in km
  float dLat = (lat2 - lat1) * (PI / 180.0f);
  float dLon = (lon2 - lon1) * (PI / 180.0f);
  float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
            cos(lat1 * (PI / 180.0f)) * cos(lat2 * (PI / 180.0f)) *
            sin(dLon / 2.0f) * sin(dLon / 2.0f);
  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  return R * c; // km
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = (lon2 - lon1) * (PI / 180.0f);
  float y = sin(dLon) * cos(lat2 * (PI / 180.0f));
  float x = cos(lat1 * (PI / 180.0f)) * sin(lat2 * (PI / 180.0f)) -
            sin(lat1 * (PI / 180.0f)) * cos(lat2 * (PI / 180.0f)) * cos(dLon);
  float brng = atan2(y, x) * (180.0f / PI);
  return brng < 0 ? brng + 360.0f : brng; // degrees
}

void FCMS::launchDirectionHold(float roll, float rollrate) {
  float rollThreshold = 5.0;    // 5 degrees
  float rollRateThreshold = 5.0; // 5 degrees per second

  if (fabs(roll) > rollThreshold || fabs(rollrate) > rollRateThreshold) {
    Serial.print("Adjusting roll to: ");
    Serial.print(roll, 2);
    Serial.println(" degrees");

    Serial.print("Adjusting roll rate to: ");
    Serial.print(rollrate, 2);
    Serial.println(" degrees per second");
  } else {
    currentMode = WAYPOINT_GUIDANCE;
  }
}

void FCMS::navigateToWaypoint(Position currentPos, Waypoint targetPos) {
  float distance = haversine(currentPos.lat, currentPos.lon, targetPos.lat, targetPos.lon);
  float bearing = calculateBearing(currentPos.lat, currentPos.lon, targetPos.lat, targetPos.lon);

  Serial.print("Distance to waypoint: ");
  Serial.print(distance, 2);
  Serial.println(" km");

  Serial.print("Bearing to waypoint: ");
  Serial.print(bearing, 2);
  Serial.println(" degrees");

  float waypointThreshold = 0.03f; // 30 meters
  float maxPitchLimit = 20.0f;    // 20 degrees

  if (distance < waypointThreshold || fabs(imu_.getAnglePitch()) > maxPitchLimit) {
    currentMode = ABORT_EULER;
  } else {
    adjustPitchAndYaw(bearing, imu_.getAnglePitch());
  }
}

void FCMS::abortEuler(float pitch, float roll) {
  float pitchThreshold = 5.0;  // 5 degrees
  float rollThreshold = 5.0;   // 5 degrees

  float pitchError = (fabs(pitch) > pitchThreshold) ? pitch - copysign(pitchThreshold, pitch) : 0.0f;
  float rollError = (fabs(roll) > rollThreshold) ? roll - copysign(rollThreshold, roll) : 0.0f;

  if (fabs(pitchError) > 0 || fabs(rollError) > 0) {
    Serial.print("Adjusting pitch to: ");
    Serial.print(pitchError, 2);
    Serial.println(" degrees");

    Serial.print("Adjusting roll to: ");
    Serial.print(rollError, 2);
    Serial.println(" degrees");
  } else {
    Serial.println("Rocket stabilized. Ready for parachute deployment.");
  }
}

void FCMS::adjustPitchAndYaw(float bearing, float pitch) {
  float pitchThreshold = 5.0;  // 5 degrees
  float yawThreshold = 5.0;    // 5 degrees

  float pitchError = pitch - pitchThreshold;
  float yawError = bearing - yawThreshold;

  if (fabs(pitchError) > pitchThreshold || fabs(yawError) > yawThreshold) {
    Serial.print("Adjusting pitch to: ");
    Serial.print(pitchError, 2);
    Serial.println(" degrees");

    Serial.print("Adjusting yaw to: ");
    Serial.print(yawError, 2);
    Serial.println(" degrees");
  } else {
    Serial.println("Pitch and yaw aligned.");
  }
}


void FCMS::waypoint_gps() {
  float currentLat = gps_.getLatitude();
  float currentLon = gps_.getLongitude();

  Position currentPos{currentLat, currentLon};
  Waypoint waypoint{49.8419f, 24.0316f};

  float angleRoll_ = imu_.getAngleRoll();
  float angleRollrate_ = imu_.getRollRate();
  float anglePitch_ = imu_.getAnglePitch();
  

switch (currentMode) {
  case LAUNCH_DIRECTION_HOLD:
    launchDirectionHold(angleRoll_, angleRollrate_);
    break;

  case WAYPOINT_GUIDANCE:
    navigateToWaypoint(currentPos, waypoint);
    break;

  case ABORT_EULER:
    abortEuler(angleRoll_, anglePitch_);
    break;
  
  default:
  Serial.println("Unknown mode");
    break;
  } 
}


void FCMS::test_waypoint() {
    static int testScenario = 0;  
    static float pitch = 0.0, roll = 0.0;
    static Position currentPos = {49.8419f, 24.0315f};
    static Waypoint waypoint = {49.8419f, 24.0316f};
    static float simulatedDistance = 0.05f;  // 50 meters

    Serial.print("Current Mode: ");
    Serial.println(currentMode);

    switch (testScenario) {
        case 0:  // Stable launch
            Serial.println("Scenario 0: Stable Launch");
            pitch = 0.0;
            roll = 0.0;
            break;

        case 1:  // Test roll deviation beyond threshold
            Serial.println("Scenario 1: Roll deviation");
            roll = 7.0;  // Simulate roll deviation
            break;

        case 2:  // Test waypoint proximity
            Serial.println("Scenario 2: Near waypoint");
            simulatedDistance = 0.02f;  // Simulate approaching the waypoint
            break;

        case 3:  // Test pitch exceeding limits
            Serial.println("Scenario 3: Pitch exceeds limit");
            pitch = 12.0;  // Exceed max pitch
            break;

        case 4:  // Test abort to Euler state
            Serial.println("Scenario 4: Abort Euler");
            pitch = 15.0;  // Large pitch to trigger abort
            roll = 10.0;   // Large roll to trigger abort
            break;

        default:
            Serial.println("End of scenarios.");
            return;  // Stop testing
    }

    // Update mode based on current simulated values
    switch (currentMode) {
        case LAUNCH_DIRECTION_HOLD:
            launchDirectionHold(pitch, roll);
            break;

        case WAYPOINT_GUIDANCE:
            // Simulate navigation and check distance
            currentPos = {49.8419f, 24.0315f};  // Simulate current position
            waypoint = {49.8420f, 24.0316f};    // Update waypoint if needed
            navigateToWaypoint(currentPos, waypoint);
            break;

        case ABORT_EULER:
            abortEuler(pitch, roll);
            break;

        default:
            Serial.println("Unknown mode");
            break;
    }

    static unsigned long lastTime = 0;
    if (millis() - lastTime > 5000) {
        lastTime = millis();
        testScenario++;
    }
}
