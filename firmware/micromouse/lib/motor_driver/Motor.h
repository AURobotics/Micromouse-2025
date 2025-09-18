#pragma once
#include "Arduino.h"
#include "pins.h"

enum class Direction : uint8_t { FORWARD = 0, BACKWARD = 1, STOP = 2 };

struct Motor {
    Motor(MOTOR_PINS in1, MOTOR_PINS in2);
    // 0 to 1024
    void move(uint16_t duty, Direction direction = Direction::FORWARD) const;

private:
    MOTOR_PINS IN1;
    MOTOR_PINS IN2;
};
