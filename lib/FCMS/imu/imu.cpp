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

void IMU::printScaledAccelData()
{
  Serial.print("AccelX:");
  Serial.print(accX_cal_);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accY_cal_);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accZ_cal_);
  Serial.println("");
}


float IMU::getRollRate()
{
  return roll_ - rollCalibration_;
}

float IMU::getPitchRate()
{
  return pitch_ - pitchCalibration_;
}

float IMU::getYawRate()
{
  return yaw_ - yawCalibration_;
}

void IMU::calibrate()
{
  Serial.print("Calibrating imu...");

  calibrateGyro();
  // calibrateAccel(0.0, 0.0, 0.0);
  calibrateAccel(0.052074, -0.026193, -0.223160);
}

void IMU::calibrateGyro()
{
  for (size_t i = 0; i < 2000; ++i) {
    updateGyro();
    rollCalibration_ += roll_;
    pitchCalibration_ += pitch_;
    yawCalibration_ += yaw_;
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

  accX_ = static_cast<float>(AccXLBS) / 4096.0 - accXCalibration_;
  accY_ = static_cast<float>(AccYLBS) / 4096.0 - accYCalibration_;
  accZ_ = static_cast<float>(AccZLBS) / 4096.0 - accZCalibration_;

  accX_cal_ = 1.003195 * accX_ + 0.001268 * accY_ + 0.001367 * accZ_;
  accY_cal_ = 0.001268 * accX_ + 0.993899 * accY_ - 0.000891 * accZ_;
  accZ_cal_ = 0.001367 * accX_ - 0.000891 * accY_ + 0.977694 * accZ_;
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

  roll_ = static_cast<float>(GyroX) / 65.5;
  pitch_ = static_cast<float>(GyroY) / 65.5;
  yaw_ = static_cast<float>(GyroZ) / 65.5;
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
  if (abs(roll_) > gyroThreshold || abs(pitch_) > gyroThreshold || abs(yaw_) > gyroThreshold) {
    Serial.println("Takeoff detected based on gyroscope data!");
    enterFlightMode();
  }
}

void IMU::enterFlightMode() {
  Serial.println("Entering flight mode...");
  // Implement any mode changes here, e.g., adjusting sensor readings or control behavior.
  flightMode = true; // Add a boolean variable to track flight state
}
