#include "FCMS.h"


void FCMS::setState(STATE state)
{
  curr_state_ = state;
}

bool FCMS::nextState()
{
  if (curr_state_ < 5) {
    curr_state_ = static_cast<STATE>(static_cast<int>(curr_state_) + 1);;
    return true;
  }
  return false;
}

STATE FCMS::getState()
{
  return curr_state_;
}
