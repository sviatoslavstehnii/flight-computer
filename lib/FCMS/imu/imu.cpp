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
  // personal rates, can be changed
  calibrateAccel(-0.03, -0.01, 0.14);
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
