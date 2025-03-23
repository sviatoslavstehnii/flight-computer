enum FlightEventStatus
{
    INACTIVE = 0,
    IN_PROGRESS = 1,
    SUCCESS = 2,
    FAILED = 3
};

class FlightEvents
{
private:
    bool takeoffDetected = false;
    bool apogeeDetected = false;
    bool landed = false;
    bool parachute_fired = false;
    bool logged_to_sd = false;
    FlightEventStatus imuCalib = INACTIVE;
    FlightEventStatus finTest = INACTIVE;
    FlightEventStatus control = INACTIVE;
    FlightEventStatus guidance_test = INACTIVE;
    uint16_t takeoffDetectedTime = 0;
    uint16_t parachuteDeployedTime = 0;
    uint16_t missionTime = 0;
    // bool FlightEventStatus

public:
    void setTakeoffDetected(bool value) { takeoffDetected = value; }
    bool getTakeoffDetected() { return takeoffDetected; }

    void setApogeeDetected(bool value) { apogeeDetected = value; }
    bool getApogeeDetected() { return apogeeDetected; }

    void setLanded(bool value) { landed = value; }
    bool getLanded() { return landed; }

    void setParachuteFired(bool value) { parachute_fired = value; }
    bool getParachuteFired() { return parachute_fired; }

    void setLoggedToSD(bool value) { logged_to_sd = value; }
    bool getLoggedToSD() { return logged_to_sd; }

    void setImuCalib(FlightEventStatus value) { imuCalib = value; }
    FlightEventStatus getImuCalib() { return imuCalib; }

    void setFinTest(FlightEventStatus value) { finTest = value; }
    FlightEventStatus getFinTest() { return finTest; }

    void setControl(FlightEventStatus value) { control = value; }
    FlightEventStatus getControl() { return control; }

    void setGuidanceTest(FlightEventStatus value) { guidance_test = value; }
    FlightEventStatus getGuidanceTest() { return guidance_test; }

    void setTakeoffDetectedTime(uint16_t value) { takeoffDetectedTime = value; }
    uint16_t getTakeoffDetectedTime() { return takeoffDetectedTime; }

    void setParachuteDeployedTime(uint16_t value) { parachuteDeployedTime = value; }
    uint16_t getParachuteDeployedTime() { return parachuteDeployedTime; }

    void setMissionTime(uint16_t value) { missionTime = value; }
    uint16_t getMissionTime() { return missionTime; }

    void clear()
    {
        takeoffDetected = false;
        apogeeDetected = false;
        landed = false;
        parachute_fired = false;
        logged_to_sd = false;
        imuCalib = INACTIVE;
        finTest = INACTIVE;
        control = INACTIVE;
        guidance_test = INACTIVE;
        takeoffDetectedTime = 0;
        parachuteDeployedTime = 0;
        missionTime = 0;
    }
};