#include "pyro.h"

// Pyro::Pyro(int pin) {
//   _pin = pin;
//   pinMode(_pin, OUTPUT);
//   digitalWrite(_pin, LOW);
//   _state = DISARMED;
// }

bool Pyro::check_continuity() {
  _state = CONTINUITY;
  return false;
}

bool Pyro::fire() {
  if (_state != ARMED) {
    return false;
  }

  digitalWrite(_pin, HIGH);
  return true;
}

bool Pyro::arm() {
  _state = ARMED;
  return true;
}

bool Pyro::disarm() {
  _state = DISARMED;
  return true;
}