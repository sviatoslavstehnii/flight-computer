#include "kalman_filter.h"



void KalmanFilter::updateRoll(float input, float measurment)
{
  // roll update
  angleRoll_ = angleRoll_ + dt_*input;
  uncertaintyAngleRoll_ = uncertaintyAngleRoll_ + dt_ * dt_ * 4 * 4;
  float KalmanGain = uncertaintyAngleRoll_ * 1/(1*uncertaintyAngleRoll_ + 3*3);
  angleRoll_ = angleRoll_ + KalmanGain* (measurment - angleRoll_);
  uncertaintyAngleRoll_ = (1-KalmanGain)*uncertaintyAngleRoll_;
}

void KalmanFilter::updatePitch(float input, float measurment)
{
  // pitch update
  anglePitch_ = anglePitch_ + dt_*input;
  uncertaintyAnglePitch_ = uncertaintyAnglePitch_ + dt_ * dt_ * 4 * 4;
  float KalmanGain = uncertaintyAnglePitch_ * 1/(1*uncertaintyAnglePitch_ + 3*3);
  anglePitch_ = anglePitch_ + KalmanGain* (measurment - anglePitch_);
  uncertaintyAnglePitch_ = (1-KalmanGain)*uncertaintyAnglePitch_;
}

float KalmanFilter::getAngleRoll()
{
  return angleRoll_;
}

float KalmanFilter::getAnglePitch()
{
  return anglePitch_;
}
