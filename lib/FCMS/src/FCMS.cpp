#include "FCMS.h"

#define GROUND_STATION_ADDR 0xCC

void FCMS::setColor(int r, int g, int b)
{
#ifdef ZEST_CHIP
  CRGB color(r, g, b);
  fill_solid(main_led, 1, color);
  FastLED.show();
#else
  digitalWrite(LED_RED_PIN, r < 200 ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, g < 200 ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, b < 200 ? HIGH : LOW);
#endif
}

void FCMS::testFins()
{
  int maxDeg = 120;
  int minDeg = 60;
  fin1_servo.write(fins.fin1_offset + 90);
  fin2_servo.write(fins.fin2_offset + 90);
  fin3_servo.write(fins.fin3_offset + 90);
  fin4_servo.write(fins.fin4_offset + 90);
  delay(150);
  fin1_servo.write(minDeg + fins.fin1_offset);
  delay(150);
  fin1_servo.write(maxDeg + fins.fin1_offset);
  delay(150);
  fin1_servo.write(90 + fins.fin1_offset);
  delay(150);
  fin2_servo.write(minDeg + fins.fin2_offset);
  delay(150);
  fin2_servo.write(maxDeg + fins.fin2_offset);
  delay(150);
  fin2_servo.write(90 + fins.fin2_offset);
  delay(150);
  fin3_servo.write(minDeg + fins.fin3_offset);
  delay(150);
  fin3_servo.write(maxDeg + fins.fin3_offset);
  delay(150);
  fin3_servo.write(90 + fins.fin3_offset);
  delay(150);
  fin4_servo.write(minDeg + fins.fin4_offset);
  delay(150);
  fin4_servo.write(maxDeg + fins.fin4_offset);
  delay(150);
  fin4_servo.write(90 + fins.fin4_offset);
  delay(300);

  fin1_servo.write(fins.fin1_offset + 90);
  fin2_servo.write(fins.fin2_offset + 90);
  fin3_servo.write(fins.fin3_offset + 90);
  fin4_servo.write(fins.fin4_offset + 90);
  delay(400);
  fin1_servo.write(fins.fin1_offset + maxDeg);
  fin3_servo.write(fins.fin3_offset + minDeg);
  delay(400);
  fin1_servo.write(fins.fin1_offset + minDeg);
  fin3_servo.write(fins.fin3_offset + maxDeg);
  delay(400);
  fin1_servo.write(fins.fin1_offset + 90);
  fin3_servo.write(fins.fin3_offset + 90);
  delay(400);
  fin2_servo.write(fins.fin2_offset + maxDeg);
  fin4_servo.write(fins.fin4_offset + minDeg);
  delay(400);
  fin2_servo.write(fins.fin2_offset + minDeg);
  fin4_servo.write(fins.fin4_offset + maxDeg);
  delay(400);
  fin2_servo.write(fins.fin2_offset + 90);
  fin4_servo.write(fins.fin4_offset + 90);
  delay(400);

  for (int pos = minDeg; pos <= maxDeg; pos += 1)
  {
    fin1_servo.write(pos + fins.fin1_offset);
    fin2_servo.write(pos + fins.fin2_offset);
    fin3_servo.write(pos + fins.fin3_offset);
    fin4_servo.write(pos + fins.fin4_offset);
    delay(15);
  }
  for (int pos = maxDeg; pos >= minDeg; pos -= 1)
  {
    fin1_servo.write(pos + fins.fin1_offset);
    fin2_servo.write(pos + fins.fin2_offset);
    fin3_servo.write(pos + fins.fin3_offset);
    fin4_servo.write(pos + fins.fin4_offset);
    delay(15);
  }

  delay(500);
  fins.fin1 = 90;
  fins.fin2 = 90;
  fins.fin3 = 90;
  fins.fin4 = 90;
  fin1_servo.write(fins.fin1 + fins.fin1_offset);
  fin2_servo.write(fins.fin2 + fins.fin2_offset);
  fin3_servo.write(fins.fin3 + fins.fin3_offset);
  fin4_servo.write(fins.fin4 + fins.fin4_offset);
}

void FCMS::setup()
{
  Serial2.begin(115200);
#ifdef ZEST_CHIP
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(main_led, 1).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  Serial.println("FAST_LED Setup");
#else
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
#endif
  setColor(0, 255, 255); // light blue
  cameras.disarm();
  parachute.disarm();

  uint32_t melody_time_s = millis();
  buzzer.playMelody(MelodyType::STARTUP);
  while (millis() - melody_time_s < 1500)
  {
    buzzer.update(millis());
  }

  // delay(1000);
  Serial.println("Set up FCMS");

  fin1_servo.attach(FIN1_PIN);
  fin2_servo.attach(FIN2_PIN);
  fin3_servo.attach(FIN3_PIN);
  fin4_servo.attach(FIN4_PIN);
  delay(300);

  testFins();

  // flash_.setup(16777216, false);
  if (!sdmc_.setup())
    initialization_error = true;

  Wire.begin();
  if (!imu_.setup())
    initialization_error = true;
  if (!imu9dof_.setup())
    initialization_error = true;
  if (!baro_.setup())
    initialization_error = true;
  if (!baro388_.setup())
    initialization_error = true;
  gps_.setup();
  transceiver.setup();
  fins.roll_setp = 0;
  fins.pitch_setp = 0;
  fins.yaw_setp = 0;
  // char setup_data[] = "setup";
  // if (!flash_.writeToDJ(setup_data, sizeof(setup_data)))
  // {
  //   Serial.println("ERROR: failed to write setup data to flash chip");
  // }

  goToState(IDLE);
}

void FCMS::checkHealth()
{
  int time = millis();
  Serial.println("Check health of gps...");
  while (millis() - time < HEALTH_CHECK_TIMEOUT)
  {
    gps_.update();
    if (gps_.lat > 0.0 && gps_.lon > 0.0)
    {
      Serial.println("GPS Healthy");
    }
  }

  Serial.println("Check health of imu...");
  time = millis();
  float prev_val = 0;
  while (millis() - time < HEALTH_CHECK_TIMEOUT)
  {
    imu_.update();
    float new_val = imu_.getAccelX();
    if (new_val != prev_val && abs(prev_val) > EPSYLON)
    {
      Serial.println("IMU Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  Serial.println("Check health of imu9dof...");
  time = millis();
  prev_val = 0;
  while (millis() - time < HEALTH_CHECK_TIMEOUT)
  {
    imu9dof_.update();
    float new_val = imu9dof_.getAccelX();
    if (new_val != prev_val && abs(prev_val) > EPSYLON)
    {
      Serial.println("IMU9DOF Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  Serial.println("Check health of barometer388...");
  time = millis();
  prev_val = 0;
  while (millis() - time < HEALTH_CHECK_TIMEOUT)
  {
    baro388_.update();
    float new_val = baro388_.getAltitude();
    if (new_val != prev_val && abs(prev_val) > EPSYLON)
    {
      Serial.println("BMP388 Healthy");
      break;
    }
    prev_val = new_val;
    delay(10);
  }

  Serial.println("Check health of barometer280...");
  time = millis();
  prev_val = 0;
  while (millis() - time < HEALTH_CHECK_TIMEOUT)
  {
    baro_.update();
    float new_val = baro_.getAltitude();
    if (new_val != prev_val && abs(prev_val) > EPSYLON)
    {
      Serial.println("BMP280 Healthy");
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

  switch (state)
  {
  case IDLE:
    parachute.disarm();
    imu_.disableTakeoffDetection();
    setColor(0, 0, 255);
    buzzer.stopApogeeTune();
    break;
  case LAUNCH:
    imu_.enableTakeoffDetection();
    buzzer.playMelody(MelodyType::LAUNCH_M);
    major_events_q_.push({"READY TO LAUNCH", millis()});
    Serial.println("TAKEOFF DETECTION ENABLED");
    setColor(0, 255, 0);
    break;
  case FLIGHT:
    events.setTakeoffDetected(true);
    buzzer.playMelody(MelodyType::FLIGHT_M);
    setColor(255, 255, 0);
    major_events_q_.push({"FLIGHT", millis()});
    // sdmc_.writeLOG("FLIGHT");
    break;
  case DESCENT:
    buzzer.playMelody(MelodyType::DESCENT_M);
    setColor(255, 0, 0);
    events.setParachuteFired(true);
    events.setApogeeDetected(true);
    major_events_q_.push({"DESCENT", millis()});
    // sdmc_.writeLOG("DESCENT");
    break;
  case LANDED:
    setColor(255, 255, 255);
    // buzzer.playMelody(MelodyType::LANDED_M);
    major_events_q_.push({"LANDED", millis()});
    // sdmc_.writeLOG("LANDED");
    events.setLanded(true);

    // Start apogee tune
    buzzer.startApogeeTune(static_cast<int>(baro388_.getMaxApogee()));
    break;
  };
}

void FCMS::estimateAttitude()
{
  float rollRate, angleRoll, pitchRate, anglePitch = 0.0f;
  // first imu
  imu9dof_.update();
  sensor_data_.pitchRate1 = imu_.getPitchRate();
  sensor_data_.rollRate1 = imu_.getRollRate();
  sensor_data_.yawRate1 = imu_.getYawRate();

  // second imu
  imu_.update();
  // rollRate = imu_.getRollRate();
  // angleRoll = imu_.getAngleRoll();
  // pitchRate = imu_.getPitchRate();
  // anglePitch = imu_.getAnglePitch();
  // kf_.updateYaw(imu_.getYawRate());
  // kf_.updateRoll(rollRate, angleRoll);
  // kf_.updatePitch(pitchRate, anglePitch);

  sensor_data_.pitch2 = imu9dof_.getPitch();
  sensor_data_.roll2 = imu9dof_.getRoll();
  sensor_data_.yaw2 = imu9dof_.getYaw();

  // print for debug
  // Serial.print("pitch: ");
  // Serial.print(sensor_data_.pitch2);
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.print(sensor_data_.roll2);
  // Serial.print(", ");
  // Serial.print("yaw: ");
  // Serial.println(sensor_data_.yaw2);
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
  gps_.update();
  sensor_data_.lat = gps_.lat;
  sensor_data_.lon = gps_.lon;
}

void FCMS::commitFlash()
{

  // std::ostringstream data_msg;
  // data_msg << millis() << ",P1:" << std::fixed << std::setprecision(2) << sensor_data_.pitchRate1 << ",R1:" << std::fixed << std::setprecision(2) << sensor_data_.rollRate1
  //          << ",Y1:" << std::fixed << std::setprecision(2) << sensor_data_.yawRate1
  //          << ",Alt1:" << std::fixed << std::setprecision(1) << sensor_data_.alt1
  //          << ",Alt2:" << std::fixed << std::setprecision(1) << sensor_data_.alt2
  //          << ",P2:" << std::fixed << std::setprecision(2) << sensor_data_.pitch2 << ",R2" << std::fixed << std::setprecision(2) << sensor_data_.roll2
  //          << ",Y2:" << std::fixed << std::setprecision(2) << sensor_data_.yaw2
  //          << ",Lat:" << std::fixed << std::setprecision(6) << sensor_data_.lat
  //          << ",Lon:" << std::fixed << std::setprecision(6) << sensor_data_.lon
  //          << ",AccX:" << std::fixed << std::setprecision(2) << imu_.getAccelX()
  //          << ",AccY:" << std::fixed << std::setprecision(2) << imu_.getAccelY()
  //          << ",AccZ:" << std::fixed << std::setprecision(2) << imu_.getAccelZ()
  //          << ",AP1:" << std::fixed << std::setprecision(1) << (baro_.getMaxApogee() > 0 ? baro_.getMaxApogee() : -1)
  //          << ",AP2:" << std::fixed << std::setprecision(1) << (baro388_.getMaxApogee() > 0 ? baro388_.getMaxApogee() : -1)
  //          << ",Fin1:" << std::fixed << std::setprecision(1) << fins.fin1
  //          << ",Fin2:" << std::fixed << std::setprecision(1) << fins.fin2
  //          << ",Fin3:" << std::fixed << std::setprecision(1) << fins.fin3
  //          << ",Fin4:" << std::fixed << std::setprecision(1) << fins.fin4
  //          << ",S:" << getState() << "\n";
  // std::string data_msg_str = data_msg.str();

  // char data_buf[data_msg_str.size() + 1];
  // std::copy(data_msg_str.begin(), data_msg_str.end(), data_buf);
  // data_buf[data_msg_str.size()] = '\0';

  // if (!flash_.writeToDJ(data_buf, sizeof(data_buf)))
  // {
  //   Serial.println("error");
  // }

  // std::ostringstream data_msg2;
  // data_msg2

  // std::string data_msg_str2 = data_msg2.str();

  // char data_buf2[data_msg_str2.size() + 1];
  // std::copy(data_msg_str2.begin(), data_msg_str2.end(), data_buf2);
  // data_buf2[data_msg_str2.size()] = '\0';

  // if (!flash_.writeToDJ(data_buf2, sizeof(data_buf2)))
  // {
  //   Serial.println("Error writing data");
  // }

  // while (!major_events_q_.empty())
  // {
  //   std::pair<char *, uint32_t> event = major_events_q_.front();
  //   major_events_q_.pop();

  //   std::ostringstream event_msg;
  //   event_msg << event.first << "; " << event.second;
  //   std::string event_msg_str = event_msg.str();

  //   char event_buf[event_msg_str.size() + 1];
  //   std::copy(event_msg_str.begin(), event_msg_str.end(), event_buf);
  //   event_buf[event_msg_str.size()] = '\0';

  //   flash_.writeToMEJ(event_buf, sizeof(event_buf));
  //   Serial.println(event_buf);
  // }
}

void FCMS::commitSDMC()
{
  sdmc_.remove("dj.txt");
  size_t bytes_read = 0;
  int time = millis();
  while (millis() - time < COMMIT_TIMEOUT)
  {
    char read_data[1000] = "";
    flash_.readDJ(read_data, 1000, bytes_read);
    if (strlen(read_data) == 0)
    {
      break;
    }
    if (!sdmc_.write("dj.txt", read_data))
    {
      Serial.println("error while writing to file");
      break;
    }
    bytes_read += strlen(read_data);
  }

  sdmc_.remove("mej.txt");
  bytes_read = 0;
  time = millis();
  while (millis() - time < COMMIT_TIMEOUT)
  {
    char read_data[1000] = "";
    flash_.readMEJ(read_data, 1000, bytes_read);
    if (strlen(read_data) == 0)
    {
      break;
    }
    if (!sdmc_.write("mej.txt", read_data))
    {
      Serial.println("error while writing to file");
      break;
    }
    bytes_read += strlen(read_data);
  }
}

Response FCMS::processCommand(const Command &command)
{
  Response response{};
  for (size_t i = 0; i < sizeof(response.data); ++i)
  {
    response.data[i] = 0;
  }

  switch (command.commandId)
  {
  case PING:
    break;
  case VEHICLE_SAFE:
    parachute.disarm();
    break;
  case MASTER_ABORT:
    goToState(IDLE);
    Serial.printf("MASTER ABORT %d\n", millis());
    major_events_q_.push({"LAUNCH MASTER ABORT", millis()});
#ifdef HITL_MODE
    hitl = false;
    baro388_.stop_hitl();
    baro_.stop_hitl();
    imu_.stop_hitl();
    imu_.disableTakeoffDetection();
    baro388_.resetApogee();
    baro_.resetApogee();
#endif
    break;
  case FIN_ALIGNMENT:
  {
    if (getState() == IDLE)
    {
      // int8, int8, int8, int8	zero out(0/1), save(0/1), finsN (0-3), offset (+/-)
      int8_t zero_out = command.args[0];
      int8_t save = command.args[1];
      int8_t finsN = command.args[2];
      int8_t offset = command.args[3];
      Serial.printf("%d %d %d %d\n", zero_out, save, finsN, offset);

      if (finsN == 1)
      {
        fins.fin1_offset += offset;
        fin1_servo.write(fins.fin1 + fins.fin1_offset);
      }
      else if (finsN == 2)
      {
        fins.fin2_offset += offset;
        fin2_servo.write(fins.fin2 + fins.fin2_offset);
      }
      else if (finsN == 3)
      {
        fins.fin3_offset += offset;
        fin3_servo.write(fins.fin3 + fins.fin3_offset);
      }
      else if (finsN == 4)
      {
        fins.fin4_offset += offset;
        fin4_servo.write(fins.fin4 + fins.fin4_offset);
      }
      if (zero_out)
      {
        fins.fin1 = 90;
        fins.fin2 = 90;
        fins.fin3 = 90;
        fins.fin4 = 90;
        fin1_servo.write(fins.fin1 + fins.fin1_offset);
        fin2_servo.write(fins.fin2 + fins.fin2_offset);
        fin3_servo.write(fins.fin3 + fins.fin3_offset);
        fin4_servo.write(fins.fin4 + fins.fin4_offset);
      }
    }
    break;
  }
  case CLEAR_EVENTS:
    if (getState() == IDLE)
    {
      events.clear();
    }
    break;
  case SET_HOME:
  {
    int32_t lat = static_cast<int32_t>(gps_.getLatitude() * 1000000);
    int32_t lon = static_cast<int32_t>(gps_.getLongitude() * 1000000);
    memcpy(&response.data, &lat, sizeof(int32_t));
    memcpy(&response.data[4], &lon, sizeof(int32_t));
    break;
  }
  case TEST_FINS:
    if (getState() == IDLE)
    {
      testFins();
    }
    break;
  case CALIBRATE_IMU:
    if (getState() == IDLE)
    {
      imu_.calibrate();
      imu9dof_.calibrate();
    }
    break;
  case CALIBRATE_BARO:
    if (getState() == IDLE)
    {
      baro388_.calibrate();
      baro_.calibrate();
    }
    break;
  case HITL:
#ifdef HITL_MODE
    hitl = true;
    buzzer.playMelody(MelodyType::ALERT);
    baro388_.start_hitl();
    baro_.start_hitl();
    imu_.start_hitl();
#endif
    break;
  case VEHICLE_ARM:
    parachute.arm();
    events.setMissionTime(static_cast<uint16_t>(millis() / 1000));
    break;
  case ENABLE_TAKEOFF_DETECTION:
  {
    uint8_t flag = command.args[0];
    if (flag)
    {
      goToState(LAUNCH);
    }
    else
    {
      imu_.disableTakeoffDetection();
      goToState(IDLE);
      major_events_q_.push({"LAUNCH ABORT", millis()});
      Serial.println("TAKEOFF DETECTION DISABLED");
    }
    break;
  }
  case MANUAL_DEPLOY_PARACHUTE:
    parachute.fire();
    events.setParachuteFired(true);
    buzzer.playMelody(MelodyType::SUCCESS_M);
    major_events_q_.push({"MANUAL DEPLOY CHUTES", millis()});
    break;
  case ENABLE_RECOVERY_BUZZING:
    break;
  case ENABLE_CAMERAS:
  {
    uint8_t flag = command.args[0];
    if (flag)
    {
      cameras.arm();
      cameras.fire();
    }
    else
    {
      cameras.disarm();
    }
    break;
  }
  case DUMP_MEMORY_TO_FLASH:
    commitFlash();
    break;
  case DUMP_FLASH_TO_SD_CARD:
    commitSDMC();
    break;
  case DISABLE_CONTROL:
    break;
  case ABORT_FLIGHT_PROGRAM:
    break;
  case SEND_FLIGHT_PROGRAM:
    break;
  default:
    Serial.print("ERROR: UNDEFINED COMMAND TYPE: ");
    Serial.println(command.commandId);
    break;
  }

  return response;
}

void FCMS::step()
{
  STATE state = getState();

  prev_time_ms = time_now_ms;
  time_now_ms = millis();
  int fps = 1000 / (time_now_ms - prev_time_ms + 1);
  buzzer.update(time_now_ms);
  if (fps < 500)
  {
    // Serial.printf("FPS: %d\n", fps);
  }

  // if (state != SAFE && state != LANDED) {
  if (time_now_ms - estimateAltitudeMillis >= estimateAltitudeInterval)
  {
    estimateAltitudeMillis = time_now_ms;
    estimateAltitude();
    estimateAttitude();

    // 0 degrees deflection -- idle
    float roll_diff = sensor_data_.roll2 - fins.roll_setp;
    int f1a = fins.fin1 + fins.fin1_offset - 90;
    int f2a = fins.fin2 + fins.fin2_offset - 90;
    int f3a = fins.fin3 + fins.fin3_offset - 90;
    int f4a = fins.fin4 + fins.fin4_offset - 90;
    // f(alpha) = surface depending on alpha
    float roll_rot_force = sin(f1a) * cos(f1a) + sin(f2a) * cos(f2a) + sin(f3a) * cos(f3a) + sin(f4a) * cos(f4a);
    float roll_antirot_force;
    int f1a_sp;

    float Kp = 0.05;
    float roll_rate_error = std::fmod(std::fmod(fins.roll_setp - sensor_data_.roll2, 360.0f) + 540.0f, 360.0f) - 180.0f;
    float correction = Kp * roll_rate_error;
    float f1d = fins.fin1 + correction;
    float f2d = fins.fin2 + correction;
    float f3d = fins.fin3 + correction;
    float f4d = fins.fin4 + correction;
    float limit = 12;
    if (f1d - 90 > limit)
      f1d = 90 + limit;
    if (f1d - 90 < -limit)
      f1d = 90 - limit;
    if (f2d - 90 > limit)
      f2d = 90 + limit;
    if (f2d - 90 < -limit)
      f2d = 90 - limit;
    if (f3d - 90 > limit)
      f3d = 90 + limit;
    if (f3d - 90 < -limit)
      f3d = 90 - limit;
    if (f4d - 90 > limit)
      f4d = 90 + limit;
    if (f4d - 90 < -limit)
      f4d = 90 - limit;
    fin1_servo.writeMicroseconds(map(f1d + fins.fin1_offset, 0, 180, 1000, 2000));
    fin2_servo.writeMicroseconds(map(f2d + fins.fin2_offset, 0, 180, 1000, 2000));
    fin3_servo.writeMicroseconds(map(f3d + fins.fin3_offset, 0, 180, 1000, 2000));
    fin4_servo.writeMicroseconds(map(f4d + fins.fin4_offset, 0, 180, 1000, 2000));
  }
  if (time_now_ms - commsMillis >= commsInterval)
  {
    commsMillis = time_now_ms;
    transceiver.receive();
    // server.receive();

    if (transceiver.hasCommand())
    {
      auto packetRx = transceiver.popCommand();
      // process command
      Serial.println("Received command\n\n\n\n\n\n\n");
      // send response
      Response response = processCommand(packetRx.command);
      response.commandSeqId = packetRx.sequenceId;
      Serial.println(response.commandSeqId);

      transceiver.sendResponse(GROUND_STATION_ADDR, response);
    }
    // if (server.hasCommand())
    // {
    //   auto packetRx = server.popCommand();
    //   // process command
    //   Serial.printf("Received command: %d\n", packetRx.command.commandId);
    //   // send response
    //   Response response = processCommand(packetRx.command);
    //   response.commandSeqId = packetRx.sequenceId;
    //   Serial.println(response.commandSeqId);

    //   // server.sendResponse(GROUND_STATION_ADDR, response);
    // }
    Telemetry tel = mapTelemetry();
    sdmc_.logTelemetry(tel);
    transceiver.sendTelemetry(GROUND_ADDR, tel);
    // server.sendTelemetry(GROUND_ADDR, tel);
  }
  if ((time_now_ms - commitMillis >= commitInterval) && dataLogingStarted)
  {
    // commitMillis = time_now_ms;
    // commitFlash();
    // Serial.println("NO LOGGING");
  }

  if (time_now_ms - estimateGPSMillis >= estimateGPSInterval)
  {
    estimateGPSMillis = time_now_ms;
    estimateGPS();
  }

  updateState();
}

void FCMS::updateState()
{
  STATE state = getState();

  switch (state)
  {
  case IDLE:
    // Serial.println("IDLE");

    if (!dataLogingStarted)
    {
      Serial.println("Start writing data to flash");
      dataLogingStarted = true;
    }
#ifdef HITL_MODE

#endif

    // cameras.arm();
    // parachute.arm();
    // imu_.enableTakeoffDetection();

    // goToState(LAUNCH);
    // setColor(0, 255, 0);

    break;

  case LAUNCH:
    // Serial.println("LAUNCH");
    if (!dataLogingStarted)
    {
      Serial.println("Start writing data to flash");
      dataLogingStarted = true;
    }
    if (firstlaunch)
    {
      launchAbortTime = millis();
      firstlaunch = false;
    }

    if (imu_.getTakeoffDetected() || baro388_.getAltitude() > 10.0f)
    {
      Serial.println("Takeoff detected!");
      events.setTakeoffDetectedTime(millis() / 100);
      goToState(FLIGHT);
    }
    break;

  case FLIGHT:
    // Serial.println("FLIGHT");

    if (baro388_.getApogeeDetected())
    {
      Serial.println("Apogee detected!");
      Serial.print("Max apogee: ");
      Serial.println(baro388_.getMaxApogee());
      goToState(DESCENT);
      parachute.fire();
      events.setParachuteDeployedTime(millis() / 100);
      cameras.fire();
    }

    break;

  case DESCENT:
  {
    // Serial.println("DESCENT");
    const float minParachuteDeploymentAltMeters = 20.0f;
    if (baro388_.getAltitude() > minParachuteDeploymentAltMeters)
    {
      // deploy parachute
      if (!parachuteDeployed)
      {
        Serial.println("Deploy parachute");
        parachuteDeployed = true;
        major_events_q_.push({"PARACHUTE_DEPLOY", millis()});
        // sdmc_.writeLOG("PARACHUTE_DEPLOY");
      }

      // landingDetectTime = millis();
    }

    // write 2 time faster
    commitInterval = 50;

    // detect landing
    if (parachuteDeployed)
    {
      imu_.detectLanding();
      if (imu_.getLandingDetected() && baro388_.getAltitude() < 20)
      {
        ++land;
      }
      else
      {
        land = 0;
      }
      if (land > 20)
      {
        Serial.println("Landing detected!");
        goToState(LANDED);
      }
    }
    break;
  }
  case LANDED:
    // Serial.println("LANDED");

    // write data to sd card
    if (!dataWrittenToSD)
    {
      // delay(2000);
      // commitFlash();
      // commitSDMC();
      sdmc_.closeLOG();
      dataWrittenToSD = true;
      events.setLoggedToSD(true);
    }
    break;
  default:
    break;
  };
}

Telemetry FCMS::mapTelemetry()
{
  Telemetry telemetry;

  telemetry.flightState = static_cast<uint8_t>(curr_state_); // conversion?
  telemetry.barometricAlt = static_cast<int16_t>(sensor_data_.alt1);
  telemetry.imuData.accel_x = static_cast<int16_t>(imu9dof_.getAccelX() * 100);
  telemetry.imuData.accel_y = static_cast<int16_t>(imu9dof_.getAccelY() * 100);
  telemetry.imuData.accel_z = static_cast<int16_t>(imu9dof_.getAccelZ() * 100);
  telemetry.imuData.velocity_x = static_cast<int16_t>(imu9dof_.getVelX() * 10);
  telemetry.imuData.velocity_y = static_cast<int16_t>(imu9dof_.getVelY() * 10);
  telemetry.imuData.velocity_z = static_cast<int16_t>(imu9dof_.getVelZ() * 10);
  // telemetry.imuData.position_x = static_cast<int16_t>(imu_.getPosX() * 10);
  // telemetry.imuData.position_y = static_cast<int16_t>(imu_.getPosY() * 10);
  // telemetry.imuData.position_z = static_cast<int16_t>(imu_.getPosZ() * 10);
  telemetry.imuData.roll = static_cast<int16_t>((sensor_data_.roll2 * 10));
  telemetry.imuData.pitch = static_cast<int16_t>((sensor_data_.pitch2 * 10));
  telemetry.imuData.yaw = static_cast<int16_t>((sensor_data_.yaw2 * 10));

  telemetry.fin1 = static_cast<uint8_t>(fins.fin1);
  telemetry.fin2 = static_cast<uint8_t>(fins.fin2);
  telemetry.fin3 = static_cast<uint8_t>(fins.fin3);
  telemetry.fin4 = static_cast<uint8_t>(fins.fin4);

  telemetry.mVBat = 0;
  telemetry.mABat = 0;
  telemetry.loadCell = 0;
  telemetry.temp = imu9dof_.getTemp();

  /// TODO
  telemetry.mejPercent = 0;
  telemetry.cjPercent = 0;
  telemetry.datajournalPercent = 0;

  // GPS Lat, Lon (multiplied by 1,000,000 for 6-digit precision)
  telemetry.gpsLat = static_cast<int32_t>(sensor_data_.lat * 1000000);
  telemetry.gpsLon = static_cast<int32_t>(sensor_data_.lon * 1000000);
  telemetry.gpsAlt = static_cast<int16_t>(gps_.getAltitude());

  telemetry.apogee = static_cast<int16_t>(baro388_.getMaxApogee());

  telemetry.flags.pyro1_cont = 0;
  telemetry.flags.pyro1_armed = cameras.isArmed();
  telemetry.flags.pyro1_fire = cameras.isFired();
  telemetry.flags.pyro2_cont = 0;
  telemetry.flags.pyro2_armed = parachute.isArmed();
  telemetry.flags.pyro2_fire = parachute.isFired();
  telemetry.flags.pyro3_cont = 0;
  telemetry.flags.pyro3_armed = 0;
  telemetry.flags.pyro3_fire = 0;
  telemetry.flags.apogee = events.getApogeeDetected();
  telemetry.flags.liftoff = events.getTakeoffDetected();
  telemetry.flags.landed = events.getLanded();
  telemetry.flags.parachute_fired = events.getParachuteFired();
  telemetry.flags.logged_to_sd = events.getLoggedToSD();
  telemetry.flags.critical = initialization_error;

  telemetry.yaw_setp = static_cast<int16_t>(fins.yaw_setp * 10);
  telemetry.pitch_setp = static_cast<int16_t>(fins.pitch_setp * 10);
  telemetry.roll_setp = static_cast<int16_t>(fins.roll_setp * 10);

  telemetry.parachuteDeploymentTime = events.getParachuteDeployedTime();
  telemetry.takeoffDetectedTime = events.getTakeoffDetectedTime();
  telemetry.event2_time = events.getMissionTime();

  return telemetry;
}