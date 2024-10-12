#include "imu.h"


void IMU::setup()
{
  // assume that Wire.begin() is already executed
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission();

  // low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // set sensitivity for accel
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // set sensitivity for gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  calibrate();
}


// currently prints roll, pitch, yaw
void IMU::printGyroData()
{
  Serial.print("roll=");
  Serial.println(getRollRate());
  Serial.print("pitch=");
  Serial.println(getPitchRate());
  Serial.print("yaw=");
  Serial.println(getYawRate());
  Serial.println("");
}

void IMU::printAccelData()
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


float IMU::getRollRate()
{
  return rollRate_;
}

float IMU::getPitchRate()
{
  return pitchRate_;
}

float IMU::getYawRate()
{
  return yawRate_;
}

float IMU::getAngleRoll()
{
  return angleRoll_;
}

float IMU::getAnglePitch()
{
  return anglePitch_;
}

void IMU::calibrate()
{
  Serial.print("Calibrating imu...");

  calibrateGyro();
  // calibrateAccel(-0.052074, 0.026193, 0.223160);
  calibrateAccel(-0.03, -0.01, 0.14);

}

void IMU::calibrateGyro()
{
  for (size_t i = 0; i < 2000; ++i) {
    updateGyro();
    rollCalibration_ += rollRate_;
    pitchCalibration_ += pitchRate_;
    yawCalibration_ += yawRate_;
    delay(1);
  }

  rollCalibration_ /= 2000;
  pitchCalibration_ /= 2000;
  yawCalibration_ /= 2000;
}


// highly recommended to adjust for yourself
void IMU::calibrateAccel(float xc, float yc, float zc)
{
  accXCalibration_ = xc;
  accYCalibration_ = yc;
  accZCalibration_ = zc;
}

void IMU::update()
{
  updateAccel();
  updateGyro();

  detectTakeoff();

}

void IMU::updateAccel()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t AccXLBS = Wire.read() << 8 | Wire.read();
  int16_t AccYLBS = Wire.read() << 8 | Wire.read();
  int16_t AccZLBS = Wire.read() << 8 | Wire.read();

  accX_ = static_cast<float>(AccXLBS) / 4096.0 + accXCalibration_;
  accY_ = static_cast<float>(AccYLBS) / 4096.0 + accYCalibration_;
  accZ_ = static_cast<float>(AccZLBS) / 4096.0 + accZCalibration_;

  anglePitch_ = 180 * atan2(accX_, sqrt(accY_*accY_ + accZ_*accZ_))/PI;
  angleRoll_ = 180 * atan2(accY_, sqrt(accX_*accX_ + accZ_*accZ_))/PI;
}

void IMU::updateGyro()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  rollRate_ = static_cast<float>(GyroX) / 65.5 - rollCalibration_; 
  pitchRate_ = static_cast<float>(GyroY) / 65.5 - pitchCalibration_;
  yawRate_ = static_cast<float>(GyroZ) / 65.5 - yawCalibration_;
}

void IMU::detectTakeoff() {
  // Thresholds for vibration detection
  const float accelThreshold = 2.0; // Adjust this value based on testing
  const float gyroThreshold = 200.0; // For detecting strong rotational changes

  // Check for strong acceleration
  if (abs(accX_) > accelThreshold || abs(accY_) > accelThreshold || abs(accZ_) > accelThreshold) {
    Serial.println("Takeoff detected based on acceleration!");
    enterFlightMode();
  }

  // Check for strong rotational changes
  if (abs(rollRate_) > gyroThreshold || abs(pitchRate_) > gyroThreshold || abs(yawRate_) > gyroThreshold) {
    Serial.println("Takeoff detected based on gyroscope data!");
    enterFlightMode();
  }
}

void IMU::enterFlightMode() {
  Serial.println("Entering flight mode...");
  // Implement any mode changes here, e.g., adjusting sensor readings or control behavior.
  flightMode = true; // Add a boolean variable to track flight state
}
