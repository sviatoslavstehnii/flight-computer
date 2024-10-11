// #include <SPI.h>
// #include <SPIFlash.h>
// #include <Wire.h>
// #include "FCMS.h"

// Flash mem{10};

// void setup() {
//   Serial.begin(115200);
//   while(!Serial);

//   mem.setup(16777216, true);

//   // data should end with \n and be shorter than 100
//   char data1[] = "123456";
//   char data2[] = "aboba1bboj";
//   char data3[] = "Lorem ipsum dolor sit amet, consectetuer adipiscing elit. Aenean commodo ligula eget dolor";

//   char readData[1000] = "";
//   mem.writeToCJ(data1, sizeof(data1));
//   delay(100);
//   mem.writeToCJ(data2, sizeof(data2));
//   delay(100);
//   mem.writeToCJ(data3, sizeof(data3));


//   delay(100);

//   mem.readCJ(readData, 1000);
//   Serial.println(readData);
//   Serial.println("end");

// }


// void loop() {
//   // Your code here
// }



#include <Arduino.h>
#include <FCMS.h>

IMU imu{};
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 4;

float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4;

float Kalman1DOutput[] = {0, 0};

void kalman_1d(float KalmanState, float KalmanUncertianty,
 float KalmanInput, float KalmanMeasurment) {
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertianty = KalmanUncertianty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertianty * 1/(1*KalmanUncertianty + 3*3);
  KalmanState = KalmanState + KalmanGain* (KalmanMeasurment - KalmanState);
  KalmanUncertianty = (1-KalmanGain)*KalmanUncertianty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertianty;
 }


void setup() {
  Serial.begin(115200);
  while(!Serial);
  // Print log
  Serial.println("setup");
  Wire.begin();
  delay(200);

  imu.setup();
  delay(1000);
}

uint32_t loopTimer =0;
void loop() {

  imu.update();

  // kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll,
  //  imu.getRollRate(), imu.getAngleRoll());
  // KalmanAngleRoll = Kalman1DOutput[0];
  // KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
  //  imu.getPitchRate(), imu.getAnglePitch());
  // KalmanAnglePitch = Kalman1DOutput[0];
  // KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    
  Serial.print("Angle roll: ");
  Serial.println(imu.getAngleRoll());

  Serial.print("Rate roll: ");
  Serial.println(imu.getRollRate());
  while( micros() - loopTimer  < 4000);
  loopTimer = micros(); 
}