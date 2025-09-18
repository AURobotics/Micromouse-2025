#include "IR.h"

void IR::setup() const {
    pinMode(trig_pin, OUTPUT);
    pinMode(static_cast<uint8_t>(echo_pin), INPUT);
}

// Rolling average on 10 IRs readings, will handle the starting when there are less than 10 readings
uint16_t IR::read() {
    digitalWrite(this->trig_pin, LOW);
    delayMicroseconds(5);
    const uint16_t dark_val = analogRead(static_cast<uint8_t>(this->echo_pin));
    digitalWrite(this->trig_pin, HIGH);
    delayMicroseconds(5);
    const uint16_t lit_val = analogRead(static_cast<uint8_t>(this->echo_pin));
    digitalWrite(this->trig_pin, LOW);
    // if (m_queue.size() < 10) {
    //     m_queue.push(val);
    //     m_sum += val;
    //     return m_sum * 10 / m_queue.size();
    // }
    // m_queue.push(val);
    // m_sum += val;
    // m_sum -= m_queue.front();
    // m_queue.pop();
    return (lit_val > dark_val) ? (lit_val - dark_val) : 0;
}

void IR::calibrate() {
    digitalWrite(this->trig_pin, HIGH);
    for (uint8_t i = 0; i < 100; i++) {
        this->m_threshold += analogRead(this->trig_pin);
        vTaskDelay(1);
    }
    m_threshold /= 100;
    digitalWrite(this->trig_pin, LOW);
    m_threshold = static_cast<uint16_t>(static_cast<float>(m_threshold) * 0.9f);//0.9 is a safety margin for wall or no wall
}
