#include <Wire.h>
class IMU {
  private:
    float accX_, accY_, accZ_;
    float accXCalibration_, accYCalibration_, accZCalibration_;
    float accX_cal_, accY_cal_, accZ_cal_;

    float rollRate_ = 0;
    float pitchRate_ = 0;
    float yawRate_ = 0;
    float rollCalibration_ = 0;
    float pitchCalibration_ = 0;
    float yawCalibration_ = 0;

    float angleRoll_ = 0;
    float anglePitch_ = 0;
    float angleYaw_ = 0;

    
    void calibrate();
    void calibrateGyro();
    void calibrateAccel(float xc, float yc, float zc);


    const float accelThreshold = 2.0;
    const float gyroThreshold = 200.0;
    
    const int requiredChecks = 5;
    int successCount = 0;

    bool takeoffDetected = false;
    bool landingDetected = false;


    void detectTakeoff();

  public:
    IMU() = default;
    ~IMU() = default;

    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    void setup();

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

    float getAccelX();

    void detectLanding();
    
    bool getTakeoffDetected() { return takeoffDetected; }
    bool getLandingDetected() { return landingDetected; }
};
