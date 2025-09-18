#pragma once
#include "Arduino.h"

enum class BTN_PINS { BTN1 = GPIO_NUM_11, BTN2 = GPIO_NUM_12 };

enum class ADC_PINS {
    ADC1_0 = GPIO_NUM_1,
    ADC1_1 = GPIO_NUM_2,
    ADC1_2 = GPIO_NUM_3,
    ADC1_3 = GPIO_NUM_4,
    ADC1_4 = GPIO_NUM_5,
    ADC1_5 = GPIO_NUM_6
};

enum class MOTOR_PINS : uint8_t { IN1 = 13, IN2 = 14, IN3 = 15, IN4 = 17 };
enum class ENC_PINS : uint8_t { PA_1 = 7, PB_1 = 8, PA_2 = 10, PB_2 = 9 };
