#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


class IMU {
  private:
    Adafruit_MPU6050 imu_;
    sensors_event_t a_, g_, temp_;
    float rateRoll_ = 0.0;
    float ratePitch_ = 0.0;
    float rateCalibrationRoll_ = 0.0;
    float rateCalibrationPitch_ = 0.0;

    void getEvent();

    void calibrate();
    void calibrateGyro();
    void calibrateAccel();


  public:
    IMU() = default;
    ~IMU() = default;

    IMU(const IMU&) = delete;
    IMU& operator=(const IMU&) = delete;

    void setup();

    void printData();
    void printGyroData();
    void printAccelData();
    void printTempData();

    void updateRates();
    void printRates();

    float getRollRate();
    float getPitchRate();

};

