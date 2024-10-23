#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {

  private:
    float angleRoll_ = 0;
    float anglePitch_ = 0;

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

    float getAngleRoll();
    float getAnglePitch();
};

#endif