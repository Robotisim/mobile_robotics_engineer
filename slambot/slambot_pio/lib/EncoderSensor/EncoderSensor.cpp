#include "EncoderSensor.h"

EncoderSensor::EncoderSensor(int channelA, int channelB)
: _channelA(channelA), _channelB(channelB), count(0)
{}

void EncoderSensor::begin() {
    pinMode(_channelA, INPUT_PULLUP);
    pinMode(_channelB, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(_channelA), isrWrapper, this, RISING);


}

long EncoderSensor::getCount() {
    noInterrupts();
    long temp = count;
    interrupts();
    return temp;

}

void EncoderSensor::resetCount() {
    noInterrupts();
    count = 0;
    interrupts();

}

void IRAM_ATTR EncoderSensor::isrWrapper(void* arg) {
    EncoderSensor *encoder = (EncoderSensor *) arg;
    encoder->encoderISR();

}

void IRAM_ATTR EncoderSensor::encoderISR() {
    if (digitalRead(_channelB) == HIGH) {
        count--;
      } else {
        count++;

      }

}
