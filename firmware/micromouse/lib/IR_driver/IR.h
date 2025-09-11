#pragma once
#include "Arduino.h"

enum class ADC_PINS : uint8_t {
  ADC1_0 = 1, ADC1_1, ADC1_2, ADC1_3, ADC1_4, ADC1_5
};

struct IR {
  constexpr IR(const uint8_t trig_pin, const ADC_PINS pin) : trig_pin(trig_pin),
  echo_pin(pin) {}

  void setup() const;

  //TODO: Enter a critical section
  uint16_t read() const;
private:
  uint8_t trig_pin;
  ADC_PINS echo_pin;
};