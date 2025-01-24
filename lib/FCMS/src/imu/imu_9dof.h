#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU9DOF {
  private:

    Adafruit_BNO055 bno_;
    float accX_, accY_, accZ_;


    float rollRate_ = 0;
    float pitchRate_ = 0;
    float yawRate_ = 0;


    float angleRoll_ = 0;
    float anglePitch_ = 0;
    float angleYaw_ = 0;

    int8_t temp_ = 0;

    
    void calibrate();



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
    void updateTemp();

    void printGyroData();
    void printAccelData();

    float getRollRate();
    float getPitchRate();
    float getYawRate();

    float getAngleRoll();
    float getAnglePitch();

    int8_t getTemp();

    float getAccelX();

    void detectLanding();
    
    bool getTakeoffDetected() { return takeoffDetected; }
    bool getLandingDetected() { return landingDetected; }
};
