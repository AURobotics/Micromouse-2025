#pragma once
#include <queue>
#include "Arduino.h"
#include "pins.h"

struct IR {
    IR(const uint8_t trig_pin, const ADC_PINS pin) : trig_pin(trig_pin), echo_pin(pin) {}

    void setup() const;
    uint16_t read();
    void calibrate();

public:
    uint16_t m_threshold = 0;

private:
    uint8_t trig_pin;
    ADC_PINS echo_pin;
    std::queue<uint16_t> m_queue{};
    uint16_t m_sum = 0;
};
