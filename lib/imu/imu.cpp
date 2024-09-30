#include "imu.h"


void IMU::setup()
{
  if (!imu_.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setup motion detection
  imu_.setFilterBandwidth(MPU6050_BAND_5_HZ);
  imu_.setMotionDetectionThreshold(1);
  imu_.setMotionDetectionDuration(20);
  imu_.setInterruptPinLatch(true);
  imu_.setInterruptPinPolarity(true);
  imu_.setMotionInterrupt(true);

  Serial.println("");
  delay(100);

  calibrate();
}

void IMU::printData()
{
  getEvent();


  Serial.print("AccelX:");
  Serial.print(a_.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a_.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a_.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g_.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g_.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g_.gyro.z);
  Serial.println("");
}

void IMU::printGyroData()
{
  getEvent();

  
  Serial.print("GyroX:");
  Serial.print(g_.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g_.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g_.gyro.z);
  Serial.println("");
}

void IMU::printAccelData()
{
  getEvent();

  Serial.print("AccelX:");
  Serial.print(a_.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a_.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a_.acceleration.z);
}

void IMU::printTempData()
{
  getEvent();

  Serial.print("Temperature (celcius):");
  Serial.print(temp_.temperature);
}

void IMU::updateRates()
{
  getEvent();

  rateRoll_ = g_.gyro.roll - rateCalibrationRoll_;
  ratePitch_ = g_.gyro.pitch - rateCalibrationPitch_;

  rateRoll_ *= (180/PI);
  ratePitch_ *= (180/PI);

}

void IMU::printRates()
{
  Serial.print("Roll rate: ");
  Serial.print(rateRoll_);
  Serial.print(", ");
  Serial.print("Pitch rate: ");
  Serial.print(ratePitch_);
  Serial.print("\n");
}

float IMU::getRollRate()
{
  return rateRoll_;
}

float IMU::getPitchRate()
{
  return ratePitch_;
}

void IMU::calibrate()
{
  calibrateGyro();
  calibrateAccel();
}

void IMU::calibrateGyro()
{
  Serial.print("Calibrating imu...");
  for (size_t i = 0; i < 2000; ++i) {
    getEvent();
    rateCalibrationPitch_ += g_.gyro.pitch;
    rateCalibrationRoll_ += g_.gyro.roll;
    delay(1);    
  }

  rateCalibrationPitch_ /= 2000;
  rateCalibrationRoll_ /= 2000;
}

void IMU::calibrateAccel()
{
}

void IMU::getEvent()
{
  imu_.getEvent(&a_, &g_, &temp_);
}
