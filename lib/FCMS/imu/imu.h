
class IMU {
  private:
    float accX_, accY_, accZ_;
    float accXCalibration_, accYCalibration_, accZCalibration_;

    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;

    float rollCalibration_ = 0;
    float pitchCalibration_ = 0;
    float yawCalibration_ = 0;


    void calibrate();
    void calibrateGyro();
    void calibrateAccel(float xc, float yc, float zc);


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

    float getRollRate();
    float getPitchRate();
    float getYawRate();


};

