#include "imu_9dof.h"



void IMU9DOF::setup()
{
  if(!bno_.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno_.setExtCrystalUse(true);


  calibrate();

}


void IMU9DOF::printGyroData()
{
  Serial.print("roll=");
  Serial.println(getRollRate());
  Serial.print("pitch=");
  Serial.println(getPitchRate());
  Serial.print("yaw=");
  Serial.println(getYawRate());
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

void IMU9DOF::printScaledAccelData()
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


float IMU9DOF::getRollRate()
{
  return rollRate_;
}

float IMU9DOF::getPitchRate()
{
  return pitchRate_;
}

float IMU9DOF::getYawRate()
{
  return yawRate_;
}

float IMU9DOF::getAngleRoll()
{
  return angleRoll_;
}

float IMU9DOF::getAnglePitch()
{
  return anglePitch_;
}

float IMU9DOF::getAccelX() {
    return accX_cal_;
}


void IMU9DOF::calibrate()
{
  Serial.println("Calibrating IMU9DOF...");

  calibrateGyro();
  // calibrateAccel(0.0, 0.0, 0.0);
  calibrateAccel(0.052074, -0.026193, -0.223160);

}

void IMU9DOF::calibrateGyro()
{
  float tempRollCalibration_ = 0;
  float tempPitchCalibration_ = 0;
  float tempYawCalibration_ = 0;
  for (size_t i = 0; i < 2000; ++i) {

    updateGyro();
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
void IMU9DOF::calibrateAccel(float xc, float yc, float zc)
{
  accXCalibration_ = xc;
  accYCalibration_ = yc;
  accZCalibration_ = zc;
}

void IMU9DOF::update()
{
  updateAccel();
  updateGyro();
  if (!takeoffDetected) {
    detectTakeoff();
  }
}

void IMU9DOF::updateAccel()
{

  sensors_event_t event;
  bno_.getEvent(&event);


  accX_ = event.acceleration.x - accXCalibration_;
  accY_ = event.acceleration.y - accYCalibration_;
  accZ_ = event.acceleration.z - accZCalibration_;

  accX_cal_ = 1.003195 * accX_ + 0.001268 * accY_ + 0.001367 * accZ_;
  accY_cal_ = 0.001268 * accX_ + 0.993899 * accY_ - 0.000891 * accZ_;
  accZ_cal_ = 0.001367 * accX_ - 0.000891 * accY_ + 0.977694 * accZ_;

  anglePitch_ = 180 * atan2(accX_cal_, sqrt(accY_cal_*accY_cal_ + accZ_cal_*accZ_cal_))/PI;
  angleRoll_ = 180 * atan2(accY_cal_, sqrt(accX_cal_*accX_cal_ + accZ_cal_*accZ_cal_))/PI;

}

void IMU9DOF::updateGyro()
{
  sensors_event_t event;
  bno_.getEvent(&event);


  float GyroX = event.gyro.x;
  float GyroY = event.gyro.y;
  float GyroZ = event.gyro.z;

  rollRate_ = GyroX*180.0 / PI - rollCalibration_; 
  pitchRate_ = GyroY*180.0 / PI - pitchCalibration_;
  yawRate_ = GyroZ*180.0 / PI - yawCalibration_;
}

void IMU9DOF::detectTakeoff() {
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


void IMU9DOF::detectLanding() {

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

