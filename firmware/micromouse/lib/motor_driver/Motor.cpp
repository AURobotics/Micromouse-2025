#include "Motor.h"

Motor::Motor(const MOTOR_PINS in1, const MOTOR_PINS in2) : IN1(in1), IN2(in2) {
    pinMode(static_cast<uint8_t>(IN1), OUTPUT);
    pinMode(static_cast<uint8_t>(IN2), OUTPUT);
    analogWriteResolution(static_cast<uint8_t>(this->IN1), 10);
    analogWriteResolution(static_cast<uint8_t>(this->IN2), 10);
    this->move(0);
}
// duty 0 to 1024, STOP sets both outputs to HIGH
void Motor::move(const uint16_t duty, const Direction direction) const {
    if (direction == Direction::FORWARD || direction == Direction::BACKWARD) {
        analogWrite(static_cast<uint8_t>(this->IN1),
                    duty * static_cast<uint8_t>(direction));
        analogWrite(static_cast<uint8_t>(this->IN2),
                    duty * !static_cast<uint8_t>(direction));
    }
    else {
        analogWrite(static_cast<uint8_t>(this->IN1), 1024);
        analogWrite(static_cast<uint8_t>(this->IN2), 1024);
    }
}
