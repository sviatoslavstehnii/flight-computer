#include "imu_9dof.h"



void IMU9DOF::setup()
{
  if(!bno_.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while(1);
  }
  bno_.setExtCrystalUse(true);
  bno_.setMode(OPERATION_MODE_NDOF);


  // calibrate();

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
    return accX_;
}


void IMU9DOF::calibrate()
{
  Serial.println("Calibrating IMU9DOF...");
  int time = millis();
  while (millis() - time < 50000) {
    uint8_t system, gyro, accel, mag;
    bno_.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Calibration: SYS=");
    Serial.print(system);
    Serial.print(", GYRO=");
    Serial.print(gyro);
    Serial.print(", ACCEL=");
    Serial.print(accel);
    Serial.print(", MAG=");
    Serial.println(mag);
    if (system == 3 && accel >= 3 && gyro >= 1 && mag == 3) {
      break;
    }
    delay(500);
  }
}


void IMU9DOF::update()
{
  updateAccel();
  updateGyro();
  updateTemp();
  if (!takeoffDetected) {
    detectTakeoff();
  }
}

void IMU9DOF::updateAccel()
{
  imu::Vector<3> accel = bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  accX_ = accel.x();
  accY_ = accel.y();
  accZ_ = accel.z();


  anglePitch_ = 180 * atan2(accX_, sqrt(accY_*accY_ + accZ_*accZ_))/PI;
  angleRoll_ = 180 * atan2(accY_, sqrt(accX_*accX_ + accZ_*accZ_))/PI;

}

void IMU9DOF::updateGyro()
{
  imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);

  rollRate_ = euler.y() ; 
  pitchRate_ = euler.z();
  yawRate_ = euler.x();
}

void IMU9DOF::detectTakeoff() {
  bool takeOffaccelConditions = abs(accX_) > accelThreshold || 
                    abs(accY_) > accelThreshold || 
                    abs(accZ_) > accelThreshold;
  
  bool takeOffgyroConditions = abs(rollRate_) > gyroThreshold ||
                    abs(pitchRate_) > gyroThreshold ||
                    abs(yawRate_) > gyroThreshold;

  if (takeOffaccelConditions && takeOffgyroConditions) {
    takeoffDetected = true;
  }
}


void IMU9DOF::detectLanding() {

  bool landingAccelConditions = abs(accX_) < accelThreshold && 
                         abs(accY_) < accelThreshold && 
                         abs(accZ_) < accelThreshold;

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

void IMU9DOF::updateTemp(){
  temp_ = bno_.getTemp();
}

int8_t IMU9DOF::getTemp(){
  return temp_;
}

