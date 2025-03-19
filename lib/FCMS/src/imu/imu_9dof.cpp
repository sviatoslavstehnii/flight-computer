#include "imu_9dof.h"

void IMU9DOF::setup()
{
  if (!bno_.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while(1);
  }
  else
  {
    Serial.println("IMU9DOF is ready!");
  }
  bno_.setExtCrystalUse(true);
  bno_.setMode(OPERATION_MODE_NDOF);

  // calibrate();
}

void IMU9DOF::printGyroData()
{
  Serial.print("roll=");
  Serial.println(getRoll());
  Serial.print("pitch=");
  Serial.println(getPitch());
  Serial.print("yaw=");
  Serial.println(getYaw());
  Serial.println("");
}

void IMU9DOF::printAccelData()
{
  Serial.print("AccelX:");
  Serial.print(accX_);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accY_);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accZ_);
  Serial.println("");
}

float IMU9DOF::getRoll()
{
  return roll_;
}

float IMU9DOF::getPitch()
{
  return pitch_;
}

float IMU9DOF::getYaw()
{
  return yaw_;
}

float IMU9DOF::getAngleRoll()
{
  return angleRoll_;
}

float IMU9DOF::getAnglePitch()
{
  return anglePitch_;
}

float IMU9DOF::getVelX()
{
  return velX_;
}

float IMU9DOF::getVelY()
{
  return velY_;
}

float IMU9DOF::getVelZ()
{
  return velZ_;
}

float IMU9DOF::getPosX()
{
  return posX_;
}

float IMU9DOF::getPosY()
{
  return posY_;
}

float IMU9DOF::getPosZ()
{
  return posZ_;
}

float IMU9DOF::getAccelX()
{
  return accX_;
}

float IMU9DOF::getAccelY()
{
  return accY_;
}

float IMU9DOF::getAccelZ()
{
  return accZ_;
}

void IMU9DOF::calibrate()
{
  Serial.println("Calibrating IMU9DOF...");
  int time = millis();
  uint8_t system, gyro, accel, mag;
  while (millis() - time < 30000)
  {
    bno_.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Calibration: SYS=");
    Serial.print(system);
    Serial.print(", GYRO=");
    Serial.print(gyro);
    Serial.print(", ACCEL=");
    Serial.print(accel);
    Serial.print(", MAG=");
    Serial.println(mag);
    if (system == 3 && accel >= 3 && gyro >= 1 && mag == 3)
    {
      break;
    }
    delay(250);
  }

  calibrateAccel(0.0, 0.0, 0.0);
  // calibrateAccel(0.052074, -0.026193, -0.223160);
}

// highly recommended to adjust for yourself
void IMU9DOF::calibrateAccel(float xc, float yc, float zc)
{
  /// TODO: collect data
  uint32_t time = millis();
  int n = 0;
  while (millis() - time < 1000)
  {
    updateAccel();
    xc += accX_;
    yc += accY_;
    zc += accZ_;
    delay(10);
    ++n;
  }
  xc /= n;
  yc /= n;
  zc /= n;

  accXCalibration_ = xc;
  accYCalibration_ = yc;
  accZCalibration_ = zc;
}

void IMU9DOF::update()
{
  updateAccel();
  updateGyro();
  updateTemp();
  if (!takeoffDetected)
  {
    detectTakeoff();
  }
}

void IMU9DOF::updateAccel()
{
  imu::Vector<3> accel = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  accX_ = accel.x();
  accY_ = accel.y();
  accZ_ = accel.z();

  anglePitch_ = 180 * atan2(accX_, sqrt(accY_ * accY_ + accZ_ * accZ_)) / PI;
  angleRoll_ = 180 * atan2(accY_, sqrt(accX_ * accX_ + accZ_ * accZ_)) / PI;

  // Integrate acceleration to get velocity
  unsigned long currentTime = millis();
  float g = 0.981;
  float gravityX = g * sin(roll_);
  float gravityY = -g * cos(roll_) * sin(pitch_);
  float gravityZ = -g * cos(roll_) * cos(pitch_);

  // // Subtract gravity
  float correctedAccX = accX_ - gravityX;
  float correctedAccY = accY_ - gravityY;
  float correctedAccZ = accZ_ - gravityZ;
  // Delta time in seconds
  float deltaTime = (currentTime - lastTime_) / 1000.0;

  if (deltaTime > 0)
  {
    velX_ += accX_ * deltaTime;
    velY_ += accY_ * deltaTime;
    velZ_ += accZ_ * deltaTime;

    // integrate velocity to get position
    posX_ += velX_ * deltaTime;
    posY_ += velY_ * deltaTime;
    posZ_ += velZ_ * deltaTime;
  }

  lastTime_ = currentTime;
}

void IMU9DOF::updateGyro()
{
  imu::Quaternion quat = bno_.getQuat();
  auto res_euler = (quat).toEuler();

  roll_ = res_euler.x() * 180 / PI;
  pitch_ = min(res_euler.z() * 180 / PI + 90, 180);
  yaw_ = res_euler.y() * 180 / PI;
}

void IMU9DOF::detectTakeoff()
{
  bool takeOffaccelConditions = abs(accX_) > accelThreshold ||
                                abs(accY_) > accelThreshold ||
                                abs(accZ_) > accelThreshold;

  bool takeOffgyroConditions = abs(roll_) > gyroThreshold ||
                               abs(pitch_) > gyroThreshold ||
                               abs(yaw_) > gyroThreshold;

  if (takeOffaccelConditions && takeOffgyroConditions)
  {
    takeoffDetected = true;
  }
}

void IMU9DOF::detectLanding()
{

  bool landingAccelConditions = abs(accX_) < accelThreshold &&
                                abs(accY_) < accelThreshold &&
                                abs(accZ_) < accelThreshold;

  bool landingGyroConditions = abs(roll_) < gyroThreshold &&
                               abs(pitch_) < gyroThreshold &&
                               abs(yaw_) < gyroThreshold;

  if (landingAccelConditions && landingGyroConditions)
  {
    successCount++;
  }
  else
  {
    successCount = 0;
  }

  if (successCount >= requiredChecks)
  {
    landingDetected = true;
  }
}

void IMU9DOF::updateTemp()
{
  temp_ = bno_.getTemp();
}

int8_t IMU9DOF::getTemp()
{
  return temp_;
}
