
#include <Arduino.h>
#include <FCMS.h>

FCMS fcms{};
BMP280 baro{};
IMU imu{};
SDMC sdmc{};
Flash flash_{10};

bool firstlaunch = true;
bool firstAbortLoop = true;


unsigned long launchAbortTime = 0;
unsigned long landingDetectTime = 0;
unsigned long abortLoopTime = 0;

bool dataWrittenToSD = false;




void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting...");
  Wire.begin();

  delay(200);
  fcms.setup();
  delay(1000);
}

uint32_t loopTimer = 0;
uint32_t commitTimer = 0; 
uint32_t frequency = 1000;

void loop() {
    // baro.update(); // Update BMP280 data and check for apogee
    // baro.printAltitude(); // Print the current altitude
    // imu.update(); // Update IMU data and check for takeoff/landing
    // imu.printGyroData();
    // imu.printAccelData();
    // delay(100); // Adjust the delay as necessary
    STATE state = fcms.getState();

    if (state != SAFE || state != LANDED) {
      if (millis() - commitTimer > frequency) {
        fcms.navigate();
        fcms.commit();
        commitTimer = millis();
      }
    }

    switch (state)
  {
  case SAFE:
    Serial.println("SAFE");

    fcms.goToState(LAUNCH);
    break;

  case IDLE:
    Serial.println("IDLE");
    // start logging data

    fcms.goToState(LAUNCH);

    break;

  case LAUNCH:

    if (firstlaunch) {
      launchAbortTime = millis();
      firstlaunch = false;
    }
    Serial.println("LAUNCH");

    // turn pyro on

    if (millis() - launchAbortTime > 5000) {
      fcms.goToState(ABORT);
    }

    if (fcms.takeoffDetected) {
      fcms.goToState(FLIGHT);
    }
    break;

  case FLIGHT:
    Serial.println("FLIGHT");

    if (imu.getAccelX() < 1.5f) {
      fcms.goToState(NO_POWER);
    }

    if (fcms.apogeeDetected) {
      fcms.goToState(DESCENT);
    }

    break;

  case NO_POWER:
    if (fcms.apogeeDetected) {
      fcms.goToState(DESCENT);
    }

    if (baro.getAltitude() < 70.0f) {
      fcms.goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
    }

    break;

  case DESCENT:

    if (baro.getAltitude() < 70.0f) {
      fcms.goToState(PARACHUTE_LANDING);
      landingDetectTime = millis();
    }

    // write 2 time faster
    frequency = 500;

    break;

  case PARACHUTE_LANDING:
    // deploy parachute

    // detect landing(only one way for it now)
    if (millis() - landingDetectTime > 15000) {
      fcms.landingDetected = true;
      fcms.goToState(LANDED);
    }
    break;
  case LANDED:
    delay(2000);

    // write data to sd card
      if (!dataWrittenToSD) {
      char readData[1000] = "";
      flash_.readDJ(readData, 1000);
      sdmc.write("test.txt", readData);
      }

    break;
  case ABORT:
    // turn pyro off

    // deploy parachute

    if (firstAbortLoop)
    {
      abortLoopTime = millis();
      firstAbortLoop = false;
    }

    if (millis() - abortLoopTime > 20000)
    {
      fcms.goToState(LANDED);
    }

    break;
  default:
    break;
  }
}

