#include "flash/flash.h"
#include "sdmc/sdmc.h"
#include "ina/ina.h"
// #include "comms/transceiver.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <queue>

#define VEHICLE_ADDR 0xEE
#define GROUND_ADDR 0xCC

#define EPSYLON 0.0001
#define BUZZER_PIN 28
#define HEALTH_CHECK_TIMEOUT 10000
#define COMMIT_TIMEOUT 5000

// #define GroundControl Serial1

// Ground Station Managment System
enum STATE
{
  SAFE = 1,
  ARM = 2,
  LAUNCH = 3,
};

class GSMS
{
private:
  Flash flash_;
  SDMC sdmc_{};
  // SoftwareSerial mySerial{15, 14};
  Transceiver lora{Serial3, GROUND_ADDR};
  Transceiver server{Serial1, GROUND_ADDR};

  int land = 0;

  STATE curr_state_;

  std::chrono::time_point<std::chrono::steady_clock> prevTime{};
  std::chrono::time_point<std::chrono::steady_clock> time{};

  uint32_t commitMillis = 0;
  uint32_t commitInterval = 100;

  uint32_t commsMillis = 0;
  uint32_t commsInterval = 5;

  uint32_t monitorMillis = 0;
  uint32_t monitorInterval = 400;

  uint32_t buzzerMillis = 0;
  uint32_t sequence_id = 0;

  bool dataLogingStarted = false;
  bool launched = false;
  bool pyroOn = false;
  bool dataWrittenToSD = false;

  unsigned long launchAbortTime = 0;

public:
  GSMS() : flash_(10) {}

  ~GSMS() = default;

  void setup();
  void checkHealth();
  void step();
  void updateState();

  STATE getState();
  void goToState(STATE state);

  void commitFlash();
  void commitSDMC();

  void buzzMillis(uint32_t ms);

  void sendCommand(COMMAND_ID command);
};