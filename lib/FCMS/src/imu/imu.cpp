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

float IMU::getAccelX() {
    return accX_cal_;
}


void IMU::calibrate()
{
  Serial.println("Calibrating imu...");

  calibrateGyro();
  // calibrateAccel(0.0, 0.0, 0.0);
  calibrateAccel(0.052074, -0.026193, -0.223160);

}

void IMU::calibrateGyro()
{
  float tempRollCalibration_ = 0;
  float tempPitchCalibration_ = 0;
  float tempYawCalibration_ = 0;
  for (size_t i = 0; i < 2000; ++i) {

    updateGyro();
    // Serial.println(rollRate_);
    tempRollCalibration_ += rollRate_;
    tempPitchCalibration_ += pitchRate_;
    tempYawCalibration_ += yawRate_;
    delay(1);
  }

  rollCalibration_ = tempRollCalibration_/2000;
  pitchCalibration_ = tempPitchCalibration_/2000;
  yawCalibration_ = tempYawCalibration_/2000;

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
  if (!takeoffDetected) {
    detectTakeoff();
  }
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

  anglePitch_ = 180 * atan2(accX_cal_, sqrt(accY_cal_*accY_cal_ + accZ_cal_*accZ_cal_))/PI;
  angleRoll_ = 180 * atan2(accY_cal_, sqrt(accX_cal_*accX_cal_ + accZ_cal_*accZ_cal_))/PI;

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
  bool takeOffaccelConditions = abs(accX_cal_) > accelThreshold || 
                    abs(accY_cal_) > accelThreshold || 
                    abs(accZ_cal_) > accelThreshold;
  
  bool takeOffgyroConditions = abs(rollRate_) > gyroThreshold ||
                    abs(pitchRate_) > gyroThreshold ||
                    abs(yawRate_) > gyroThreshold;

  if (takeOffaccelConditions && takeOffgyroConditions) {
    takeoffDetected = true;
  }
}


void IMU::detectLanding() {

  bool landingAccelConditions = abs(accX_cal_) < accelThreshold && 
                         abs(accY_cal_) < accelThreshold && 
                         abs(accZ_cal_) < accelThreshold;

  bool landingGyroConditions = abs(rollRate_) < gyroThreshold && 
                        abs(pitchRate_) < gyroThreshold && 
                        abs(yawRate_) < gyroThreshold;

  if (landingAccelConditions && landingGyroConditions) {
    successCount++;
  } else {
    successCount = 0;
  }

  if (successCount >= requiredChecks) {
    landingDetected = true;
  }
}

