#pragma once
#include "Arduino.h"
#include<queue>
#include "stdint-gcc.h"
#include <esp_intr_types.h>

enum class ADC_PINS : uint8_t { ADC1_0 = 1, ADC1_1, ADC1_2, ADC1_3, ADC1_4, ADC1_5 };

struct IR {
    IR(const uint8_t trig_pin, const ADC_PINS pin) : trig_pin(trig_pin), echo_pin(pin) {}

    void setup() const;
    uint16_t read();

private:
    uint8_t trig_pin;
    ADC_PINS echo_pin;
    std::queue<uint16_t> m_queue{};
    uint16_t m_sum = 0;
};
