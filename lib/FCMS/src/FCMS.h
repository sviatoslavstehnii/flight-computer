#include "pyro/pyro.h"
#include "barometer/bmp280.h"
#include "barometer/bmp180.h"
#include "barometer/bmp388.h"
#include "imu/imu.h"
#include "imu/imu_9dof.h"
#include "imu/kalman_filter.h"

#include "flash/flash.h"
#include "gps/gps.h"
#include "blackbox/blackbox.h"
#include "ina/ina.h"
#include "buzzer/buzzer.h"
#include "comms/sim7600.h"
// #include "comms/transceiver.h"
#include "flight_events.h"
#include <Servo.h>

#include <sstream>
#include <iomanip>
#include <iostream>
#include <queue>

#define VEHICLE_ADDR 0xEE
#define GROUND_ADDR 0xCC

#define EPSYLON 0.0001
#define HEALTH_CHECK_TIMEOUT 10000
#define COMMIT_TIMEOUT 5000

// #define ZEST_CHIP
#ifdef ZEST_CHIP
#include <FastLED.h>
#define PYRO_ONE_PIN 23
#define PYRO_TWO_PIN 22
#define PYRO_THREE_PIN 21
#define LED_DATA_PIN 37
#define LED_BRIGHTNESS 50
#define BUZZER_PIN 34
#else
#define PYRO_ONE_PIN 23   // Bottom, next to LORA
#define PYRO_TWO_PIN 21   // Top
#define PYRO_THREE_PIN 22 // Unavailable
#define LED_RED_PIN 25
#define LED_GREEN_PIN 26
#define LED_BLUE_PIN 27
#define BUZZER_PIN 28
#define FIN1_PIN 6
#define FIN2_PIN 3
#define FIN3_PIN 4
#define FIN4_PIN 5
#endif

// Lime Flight States
enum STATE
{
  IDLE = 0,
  LAUNCH = 1,
  FLIGHT = 2,
  DESCENT = 3,
  LANDED = 4
};

enum CURRENT_MODE
{
  LAUNCH_DIRECTION_HOLD = 0,
  ABORT_EULER = 1,
  WAYPOINT_GUIDANCE = 2
};

class FCMS
{
private:
  bool initialization_error = false;
  PyroDriver cameras{PYRO_ONE_PIN};     // Bottom, next to LORA
  PyroDriver parachute{PYRO_THREE_PIN}; // Top, next to USB
  IMU imu_{};
  IMU9DOF imu9dof_{};
  KalmanFilter kf_;
  KalmanFilter kf9dof_;
  BMP280 baro_{};
  BMP388 baro388_{};
  Flash flash_;
  Blackbox sdmc_{};
  GPS gps_{};
  Buzzer buzzer{BUZZER_PIN};
  // Transceiver transceiver{Serial3, VEHICLE_ADDR};
  SIM7600 transceiver{Serial2, VEHICLE_ADDR};
  // Transceiver server{Serial3, VEHICLE_ADDR};
  FlightEvents events{};
#ifdef ZEST_CHIP
  CRGB main_led[1];
#endif

  int land = 0;

  STATE curr_state_ = IDLE;

  struct SensorData
  {
    float pitchRate1, rollRate1, yawRate1;
    float pitch2, roll2, yaw2;
    float alt1, alt2;
    float lon, lat;
  } sensor_data_;
  float altitude_ = 0;

  struct Fins
  {
    int fin1, fin2, fin3, fin4;
    int fin1_offset = 0, fin2_offset = 0, fin3_offset = 0, fin4_offset = 0;
    float roll_start = 0, pitch_start = 0, yaw_start = 0;
    float roll_setp = 0, pitch_setp = 0, yaw_setp = 0;
  } fins;

  Servo fin1_servo;
  Servo fin2_servo;
  Servo fin3_servo;
  Servo fin4_servo;

  uint32_t estimateAltitudeMillis = 0;
  uint32_t estimateAltitudeInterval = 5;

  uint32_t estimateGPSMillis = 0;
  uint32_t estimateGPSInterval = 100;

  uint32_t commitMillis = 0;
  uint32_t commitInterval = 100;

  uint32_t commsMillis = 0;
  uint32_t commsInterval = 100;

  uint32_t monitorMillis = 0;
  uint32_t monitorInterval = 400;

  uint32_t sequence_id = 0;

  bool firstlaunch = true;
  bool firstAbortLoop = true;
  bool dataLogingStarted = false;
  bool parachuteDeployed = false;
  bool pyroOn = false;
  bool dataWrittenToSD = false;

  unsigned long launchAbortTime = 0;
  unsigned long landingDetectTime = 0;
  unsigned long abortLoopTime = 0;

  uint32_t prev_time_ms = 0;
  uint32_t time_now_ms = 0;

  bool hitl = false;

  std::queue<std::pair<char *, uint32_t>> major_events_q_;

public:
  FCMS() : flash_(10), kf_(0.04), kf9dof_(0.04) {}

  ~FCMS() = default;

  void setup();
  void checkHealth();
  void step();
  void updateState();

  STATE getState();
  void goToState(STATE state);

  void estimateAttitude();
  void estimateAltitude();
  void estimateGPS();

  void commitFlash();
  void commitSDMC();

  void testFins();

  void setColor(int r, int g, int b);

  Telemetry mapTelemetry();

  Response processCommand(const Command &command);
};