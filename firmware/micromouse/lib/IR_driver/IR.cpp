#include "IR.h"

void IR::setup() const {
    pinMode(trig_pin, OUTPUT);
    pinMode(static_cast<uint8_t>(echo_pin), INPUT);
}

// TODO: Enter a critical section
uint16_t IR::read() const {
    digitalWrite(this->trig_pin, HIGH);
    delayMicroseconds(5);
    const uint16_t val = analogRead(static_cast<uint8_t>(this->echo_pin));
    digitalWrite(this->trig_pin, LOW);
    return val;
}
