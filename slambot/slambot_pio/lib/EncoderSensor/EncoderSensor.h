#ifndef ENCODERSENSOR_H
#define ENCODERSENSOR_H

#include <Arduino.h>

class EncoderSensor {
public:
    EncoderSensor(int channelA = 6, int channelB = 4);

    void begin();
    long getCount();
    void resetCount();

private:
    int _channelA;
    int _channelB;
    volatile long count;

    void IRAM_ATTR encoderISR();
    static void IRAM_ATTR isrWrapper(void* arg);
};

#endif
