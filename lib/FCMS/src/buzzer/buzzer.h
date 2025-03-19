#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

struct Note
{
    int frequency;
    int duration;
};

enum MelodyType
{
    STARTUP,
    ALERT,
    SUCCESS_M,
    FAILURE,
    LAUNCH_M,
    FLIGHT_M,
    DESCENT_M,
    LANDED_M,
    MELODY_COUNT // Total number of melodies
};

class Buzzer
{
private:
    int buzzerPin;
    bool isPlaying = false;
    int currentMelody = STARTUP;
    int currentNote = 0;
    uint32_t noteStartTime = 0;

    bool playingApogeeTune = false;
    uint32_t apogeeTuneStartTime = 0;
    int apogeeDigits[4] = {0};
    int currentApogeeStep = -1;
    uint32_t stepStartTime = 0;
    int digitIndex = 0;
    int beepCount = 0;

    static constexpr int beepDuration = 100;
    static constexpr int beepGap = 400;
    static constexpr int digitPause = 1000;
    static constexpr int repeatPause = 3000;

public:
    Buzzer(int pin) : buzzerPin(pin)
    {
        pinMode(buzzerPin, OUTPUT);
        noTone(buzzerPin);
    }

    void playMelody(MelodyType type);
    void stopMelody();
    void update(uint32_t timeNow);
    void startApogeeTune(int apogee);
    void stopApogeeTune();
    void playApogeeTune(uint32_t timeNow);

    static const Note melodies[][20];
    static const int melodyLengths[];
};

#endif
