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
KalmanFilter kf{};

float phi_rad = 0;
float theta_rad = 0;

float P[4];
float Q[2];
float R[3];

void init(float Pinit, float * Qinit, float *Rinit) {
  P[0] = Pinit; P[1] = 0.0f;
  P[2] = Pinit; P[3] = 0.0f;

  Q[0] = Qinit[0]; Q[1] = Qinit[1];
  R[0] = Rinit[0]; R[1] = Rinit[1]; R[2] = Rinit[2];
}


void predict(float p, float q, float r, float T) {
    // Update state estimate (phi and theta angles)
    float sp = sin(phi_rad);
    float cp = cos(phi_rad);
    float tt = tan(theta_rad);

    // Update the angles using the gyroscope data (p, q, r)
    phi_rad = phi_rad + T * (p + tt * (q * sp + r * cp));
    theta_rad = theta_rad + T * (q * cp - r * sp);

    // Recalculate trigonometric terms
    sp = sin(phi_rad);
    cp = cos(phi_rad);
    float st = sin(theta_rad);
    float ct = cos(theta_rad);

    // Calculate the Jacobian matrix A based on the linearization of the system
    float A[4] = {
        tt * (q * cp - r * sp),   // A[0,0]
        (r * cp + q * sp) * (tt * tt + 1.0f),  // A[0,1]
        -(r * cp + q * sp),       // A[1,0]
        0.0f                      // A[1,1]
    };

    // Initialize Ptmp for the predicted covariance matrix
    float Ptmp[4] = {
        T * (Q[0] + 2.0f * A[0] * P[0] + A[1] * P[1] + A[1] * P[2]),
        T * (A[0] * P[1] + A[2] * P[0] + A[1] * P[3] + A[3] * P[2]),
        T * (A[0] * P[1] + A[2] * P[0] + A[1] * P[3] + A[3] * P[2]),
        T * (Q[1] + A[2] * P[1] + A[2] * P[1] + 2.0f * A[3] * P[3])
    };

    // Store the updated covariance back in P
    P[0] = Ptmp[0];
    P[1] = Ptmp[1];
    P[2] = Ptmp[2];
    P[3] = Ptmp[3];
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
  // kf.updateRoll(imu.getRollRate(), imu.getAngleRoll());
  // kf.updatePitch(imu.getPitchRate(), imu.getAnglePitch());

  // Serial.print("pitch: ");
  // Serial.print(kf.getAnglePitch());
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.println(kf.getAngleRoll());

  while(micros() - loopTimer < 4000)
  loopTimer = micros();
}

