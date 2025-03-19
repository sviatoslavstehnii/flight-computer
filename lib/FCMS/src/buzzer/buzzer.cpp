#include "buzzer.h"

const Note Buzzer::melodies[][20] = {
    // STARTUP Melody
    {{262, 200}, {330, 200}, {392, 200}, {523, 400}, {440, 500}},
    // ALERT Melody
    {{800, 300}, {600, 300}, {400, 300}, {800, 300}, {600, 300}},
    // SUCCESS Melody
    {{330, 200}, {392, 200}, {523, 400}, {660, 400}, {784, 500}},
    // FAILURE Melody
    {{400, 300}, {300, 300}, {200, 300}, {100, 500}, {0, 250}},
    // LAUNCH Melody
    {{262, 150}, {330, 150}, {392, 150}, {523, 200}, {660, 300}},
    // FLIGHT Melody
    {{500, 250}, {0, 100}, {500, 250}, {0, 100}, {500, 250}, {0, 100}, {500, 250}, {0, 100}, {500, 250}, {0, 100}, {500, 250}},
    // DESCENT Melody
    {{784, 300}, {660, 300}, {523, 300}, {392, 400}, {330, 500}},
    // LANDED Melody
    {{392, 200}, {523, 200}, {660, 400}, {523, 200}, {392, 300}}};

const int Buzzer::melodyLengths[] = {5, 5, 5, 5, 5, 11, 5, 5};

void Buzzer::playMelody(MelodyType type)
{
    currentMelody = type;
    currentNote = 0;
    isPlaying = true;
    noteStartTime = millis();
    tone(buzzerPin, melodies[currentMelody][currentNote].frequency, melodies[currentMelody][currentNote].duration);
}

void Buzzer::stopMelody()
{
    isPlaying = false;
    noTone(buzzerPin);
}

void Buzzer::update(uint32_t timeNow)
{
    if (isPlaying && currentNote < melodyLengths[currentMelody])
    {
        if (timeNow - noteStartTime >= melodies[currentMelody][currentNote].duration)
        {
            currentNote++;
            if (currentNote < melodyLengths[currentMelody])
            {
                if (melodies[currentMelody][currentNote].frequency == 0)
                {
                    noTone(buzzerPin);
                }
                else
                {
                    tone(buzzerPin, melodies[currentMelody][currentNote].frequency, melodies[currentMelody][currentNote].duration);
                }
                noteStartTime = timeNow;
            }
            else
            {
                stopMelody();
            }
        }
    }

    if (playingApogeeTune)
    {
        playApogeeTune(timeNow);
    }
}

void Buzzer::startApogeeTune(int apogee)
{
    playingApogeeTune = true;
    apogeeTuneStartTime = millis() + 5000; // Delay before start
    currentApogeeStep = -1;
    digitIndex = 0;
    beepCount = 0;

    if (apogee > 9999)
        apogee = 9999;
    if (apogee < 0)
        apogee = 0;

    for (int i = 3; i >= 0; i--)
    {
        apogeeDigits[i] = apogee % 10;
        apogee /= 10;
    }
}

void Buzzer::playApogeeTune(uint32_t timeNow)
{
    if (!playingApogeeTune)
        return;

    if (currentApogeeStep == -1)
    {
        tone(buzzerPin, 800);
        stepStartTime = timeNow;
        currentApogeeStep++;
    }
    else if (currentApogeeStep == 0 && timeNow - stepStartTime >= digitPause)
    {
        noTone(buzzerPin);
        stepStartTime = timeNow;
        currentApogeeStep++;
    }
    else if (currentApogeeStep == 1)
    {
        if (beepCount < apogeeDigits[digitIndex])
        {
            if ((timeNow - stepStartTime) % (beepDuration + beepGap) < beepDuration)
            {
                tone(buzzerPin, 1000);
            }
            else
            {
                noTone(buzzerPin);
                if ((timeNow - stepStartTime) >= (beepDuration + beepGap) * (beepCount + 1))
                {
                    beepCount++;
                }
            }
        }
        else
        {
            noTone(buzzerPin);
            stepStartTime = timeNow;
            currentApogeeStep = 2;
        }
    }
    else if (currentApogeeStep == 2 && timeNow - stepStartTime >= digitPause)
    {
        digitIndex++;
        beepCount = 0;
        if (digitIndex < 4)
        {
            currentApogeeStep = 1;
        }
        else
        {
            currentApogeeStep = 3;
            stepStartTime = timeNow;
        }
    }
    else if (currentApogeeStep == 3 && timeNow - stepStartTime >= repeatPause)
    {
        digitIndex = 0;
        beepCount = 0;
        currentApogeeStep = -1;
        stepStartTime = timeNow;
    }
}

void Buzzer::stopApogeeTune()
{
    playingApogeeTune = false;
    noTone(buzzerPin);
}
