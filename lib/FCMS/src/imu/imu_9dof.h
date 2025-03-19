#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU9DOF
{
private:
  Adafruit_BNO055 bno_;
  float accX_, accY_, accZ_;
  float accXCalibration_, accYCalibration_, accZCalibration_;
  float accX_cal_, accY_cal_, accZ_cal_;

  float velX_ = 0.0;
  float velY_ = 0.0;
  float velZ_ = 0.0;

  float posX_ = 0.0;
  float posY_ = 0.0;
  float posZ_ = 0.0;
  unsigned long lastTime_ = 0;

  float roll_ = 0;
  float pitch_ = 0;
  float yaw_ = 0;

  float angleRoll_ = 0;
  float anglePitch_ = 0;
  float angleYaw_ = 0;

  int8_t temp_ = 0;

  const float accelThreshold = 2.0;
  const float gyroThreshold = 200.0;

  const int requiredChecks = 5;
  int successCount = 0;

  bool takeoffDetected = false;
  bool landingDetected = false;

  void detectTakeoff();

public:
#ifdef ZEST_CHIP
  IMU9DOF() : bno_(55, 0x29, &Wire) {};
#else
  IMU9DOF() : bno_(55, 0x28, &Wire) {};
#endif
  ~IMU9DOF() = default;

  IMU9DOF(const IMU9DOF &) = delete;
  IMU9DOF &operator=(const IMU9DOF &) = delete;

  void setup();
  void calibrate();
  void calibrateGyro();
  void calibrateAccel(float xc, float yc, float zc);

  void update();
  void updateAccel();
  void updateGyro();
  void updateTemp();

  void printGyroData();
  void printAccelData();

  float getRoll();
  float getPitch();
  float getYaw();

  float getAngleRoll();
  float getAnglePitch();

  int8_t getTemp();

  float getAccelX();
  float getAccelY();
  float getAccelZ();

  float getVelX();
  float getVelY();
  float getVelZ();

  float getPosX();
  float getPosY();
  float getPosZ();

  void detectLanding();

  bool getTakeoffDetected() { return takeoffDetected; }
  bool getLandingDetected() { return landingDetected; }
};
