

class KalmanFilter {

  private:
    float angleRoll_ = 0;
    float anglePitch_ = 0;
    float angleYaw_ = 0;
    float filteredYawRate_ = 0; 

    float uncertaintyAngleRoll_ = 4;
    float uncertaintyAnglePitch_ = 4;

    float dt_;

  public:
    KalmanFilter(float dt): dt_{dt} {};
    ~KalmanFilter() = default;

    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    void updateRoll(float input, float measurment);
    void updatePitch(float input, float measurment);
    void updateYaw(float input);

    float getAngleRoll();
    float getAnglePitch();
    float getAngleYaw();

};