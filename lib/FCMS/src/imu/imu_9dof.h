#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU9DOF {
  private:

    Adafruit_BNO055 bno_;
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
    IMU9DOF(): bno_(55, 0x28, &Wire) {};
    ~IMU9DOF() = default;

    IMU9DOF(const IMU9DOF&) = delete;
    IMU9DOF& operator=(const IMU9DOF&) = delete;

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
