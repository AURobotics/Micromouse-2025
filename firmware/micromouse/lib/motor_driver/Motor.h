#pragma once
#include "Arduino.h"

enum class MOTOR_PINS : uint8_t { IN1 = 13, IN2 = 14, IN3 = 15, IN4 = 17 };

enum class Direction : uint8_t { FORWARD = 0, BACKWARD = 1, STOP = 2 };

struct Motor {
    Motor(MOTOR_PINS in1, MOTOR_PINS in2);
    // 0 to 1024
    void move(uint16_t duty, Direction direction = Direction::FORWARD) const;

private:
    MOTOR_PINS IN1;
    MOTOR_PINS IN2;
};
