#include <Arduino.h>
#include "pins_arduino.h"
#include <esp_adc/adc_oneshot.h>

enum class ADC_PINS : uint8_t {
    ADC1_0 = 1, ADC1_1, ADC1_2, ADC1_3, ADC1_4, ADC1_5
};

enum class MOTOR_PINS : uint8_t {
    IN1 = 13, IN2 = 14, IN3 = 15, IN4 = 17
};

enum class Direction : bool {
    FORWARD = 0, BACKWARD = 1
};

struct IR {
    constexpr IR(const uint8_t trig_pin, const ADC_PINS pin) : trig_pin(trig_pin),
    echo_pin(pin) {}
    uint8_t trig_pin;
    ADC_PINS echo_pin;
    void setup() const {
        pinMode(trig_pin, OUTPUT);
        pinMode(static_cast<uint8_t>(echo_pin), INPUT);
    }

    //TODO: Enter a critical section
    uint16_t read() const {
        digitalWrite(this->trig_pin, HIGH);
        delayMicroseconds(5);
        const uint16_t val = analogRead(static_cast<uint8_t>(this->echo_pin));
        digitalWrite(this->trig_pin, LOW);
        return val;
    }
};

struct Motor {
    Motor(const MOTOR_PINS in1, const MOTOR_PINS in2) : IN1(in1), IN2(in2) {
        // pinMode(static_cast<uint8_t>(IN1), OUTPUT);
        // pinMode(static_cast<uint8_t>(IN2), OUTPUT);
        // analogWriteResolution(static_cast<uint8_t>(this->IN1), 10);
        // analogWriteResolution(static_cast<uint8_t>(this->IN2), 10);
        // this->move(0);
    }

    void setup() const {
        pinMode(static_cast<uint8_t>(this->IN1), OUTPUT);
        pinMode(static_cast<uint8_t>(this->IN2), OUTPUT);
        analogWriteResolution(static_cast<uint8_t>(this->IN1), 10);
        analogWriteResolution(static_cast<uint8_t>(this->IN2), 10);
        this->move(0);
    }

    //0 to 1024
    void move(const uint16_t duty, const Direction direction = Direction::FORWARD) const  {
        analogWrite(static_cast<uint8_t>(this->IN1), duty * static_cast<uint8_t>(direction));
        analogWrite(static_cast<uint8_t>(this->IN2), duty * !static_cast<uint8_t>(direction));
    }

    private:
    MOTOR_PINS IN1;
    MOTOR_PINS IN2;
};

constexpr IR front_left_ir = {36, ADC_PINS::ADC1_0};
constexpr IR front_right_ir = {33, ADC_PINS::ADC1_5};
constexpr IR left_ir = {35, ADC_PINS::ADC1_2};
constexpr IR right_ir = {18, ADC_PINS::ADC1_3};
constexpr IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
constexpr IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};

constexpr std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};

const auto right_motor = Motor(MOTOR_PINS::IN1, MOTOR_PINS::IN2);
const auto left_motor = Motor(MOTOR_PINS::IN4, MOTOR_PINS::IN3);


void setup() {
    for (const auto &ir : ir_array)
        ir.setup();
    
    right_motor.setup();
    left_motor.setup();

    // right_motor.move(1024);
    // left_motor.move(1024);
}

void loop() {

}
