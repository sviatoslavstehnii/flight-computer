#include <Wire.h>
class IMU
{
private:
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

  float rollRate_ = 0;
  float pitchRate_ = 0;
  float yawRate_ = 0;
  float rollCalibration_ = 0;
  float pitchCalibration_ = 0;
  float yawCalibration_ = 0;

  float angleRoll_ = 0;
  float anglePitch_ = 0;
  float angleYaw_ = 0;

  void calibrateGyro();
  void calibrateAccel(float xc, float yc, float zc);

  const float accelThreshold = 2.0;
  const float gyroThreshold = 200.0;

  const int requiredChecks = 5;
  int successCount = 0;

  bool enableTakeoffDetection_ = false;
  bool takeoffDetected = false;
  bool landingDetected = false;

  void detectTakeoff();

// #define HITL_MODE
#ifdef HITL_MODE
  bool hitl = false;
  uint32_t start_time = 0;
  uint32_t time_now = 0;
  uint32_t motor_duration = 1050;
  uint32_t climb_duration = 11000;
  uint32_t decline_duration = 30000;
  float max_accel = 67;
#endif

public:
  IMU() = default;
  ~IMU() = default;

  IMU(const IMU &) = delete;
  IMU &operator=(const IMU &) = delete;

  bool setup();
  void calibrate();

  void update();
  void updateAccel();
  void updateGyro();

  void printGyroData();
  void printAccelData();
  void printScaledAccelData();

  float getRollRate();
  float getPitchRate();
  float getYawRate();

  float getAngleRoll();
  float getAnglePitch();
  float getAngleYaw();

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

  bool getLandingDetected() { return landingDetected; }
  bool getTakeoffDetected()
  {
    return takeoffDetected;
  }

  void enableTakeoffDetection() { enableTakeoffDetection_ = true; }
  void disableTakeoffDetection() { enableTakeoffDetection_ = false; takeoffDetected = false; }

#ifdef HITL_MODE
  void start_hitl() { hitl = true; start_time = 0; };
  void stop_hitl() { hitl = false; }
#endif
};
