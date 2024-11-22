#include "Pitches.h"
#include <Arduino.h>
class Simple_Buzzer {
private:
    int buzzer_pin;
    int channel;
public:
    void single_beep_tone(int sound_Hz, int duration)
    {
        tone(buzzer_pin, sound_Hz, duration);
        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        delay(duration);
        // stop the tone playing:
        noTone(buzzer_pin);
    }
    void play_melody_tone(int* melody, int melody_size, double* noteDurations)
    {
        // iterate over the notes of the melody:
        for (int thisNote = 0; thisNote < melody_size; thisNote++) {
            // to calculate the note duration, take one second divided by the note type.
            //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
            double noteDuration = 1000 / noteDurations[thisNote];
            tone(buzzer_pin, melody[thisNote], noteDuration);
            // to distinguish the notes, set a minimum time between them.
            // the note's duration + 30% seems to work well:
            double pauseBetweenNotes = noteDuration * 1.3;
            delay(pauseBetweenNotes);
            // stop the tone playing:
            noTone(buzzer_pin);
        }
    }
    void initialized(int pin, int _channel)
    {
        buzzer_pin=pin;
        channel = _channel;
        //ledcSetup(channel, 6000, 8);
        //ledcAttachPin(buzzer_pin, channel);
    }
    void single_beep_ledc_fade(int sound_Hz, int duration)
    {
        ledcWriteTone(channel, sound_Hz);
        int steps = ((double)duration/100);
        uint32_t dutyCycle=0;
        for(int i=0;i<100;i++)
        {
            dutyCycle=steps*(i+1);
            ledcWrite(channel, dutyCycle);
            delay(steps);
        }

    }


};
Simple_Buzzer Buzzer;