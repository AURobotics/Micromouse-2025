#include "IR.h"

#include <queue>

void IR::setup() const {
    pinMode(trig_pin, OUTPUT);
    pinMode(static_cast<uint8_t>(echo_pin), INPUT);
}

// TODO: Enter a critical section
uint16_t IR::read() {
    digitalWrite(this->trig_pin, HIGH);
    delayMicroseconds(5);
    const uint16_t val = analogRead(static_cast<uint8_t>(this->echo_pin)) / 10;
    digitalWrite(this->trig_pin, LOW);
    if (m_queue.size() != 10) {
        m_queue.push(val);
        m_sum += val;
        return m_sum * 10 / m_queue.size();
    }
    m_queue.push(val);
    m_sum += val;
    m_sum -= m_queue.front();
    m_queue.pop();
    return m_sum;
}