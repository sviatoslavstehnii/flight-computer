#include <Arduino.h>
#include <FCMS.h>

FCMS fcms{};
// test t{};
// IMU imu_{};
// KalmanFilter kf_{0.004};
// BMP280 b280{};
// BMP388 b388;
// GPS gps{};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  #ifdef MASTER_MODE
    Serial.println("Running in MASTER mode");
  #elif defined(SLAVE_MODE)
    Serial.println("Running in SLAVE mode");
  #else
    Serial.println("No mode defined. Please set build flag!");
  #endif

  Wire.begin();
  delay(200);

  fcms.setup();
  fcms.checkHealth();
  // imu_.setup();
  // b280.setup();
  // b388.setup();
  // gps.setup();

  Serial.println("STAAART");
}


void loop() {
  fcms.step();
  // if (gps.readAndParse()) {
  //   gps.printStats();
  // }

  //   imu_.update();

  // kf_.updateRoll(imu_.getRollRate(), imu_.getAngleRoll());
  // kf_.updatePitch(imu_.getPitchRate(), imu_.getAnglePitch());
  // kf_.updateYaw(imu_.getYawRate());

  // float pitch_ = kf_.getAnglePitch();
  // float roll_ = kf_.getAngleRoll();
  // float yaw_ = kf_.getAngleYaw();


  // print for debug
  // Serial.print("pitch: ");
  // Serial.print(pitch_);
  // Serial.print(", ");
  // Serial.print("roll: ");
  // Serial.print(roll_);
  // Serial.print(", ");
  // Serial.print("yaw: ");
  // Serial.println(yaw_);
}

