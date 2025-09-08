#include <Arduino.h>
#include "pins_arduino.h"
#include <esp_adc/adc_oneshot.h>

enum class ADC_PINS : uint8_t {
    ADC1_0 = 1, ADC1_1, ADC1_2, ADC1_3, ADC1_4, ADC1_5
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

constexpr IR front_left_ir = {36, ADC_PINS::ADC1_0};
constexpr IR front_right_ir = {33, ADC_PINS::ADC1_5};
constexpr IR left_ir = {35, ADC_PINS::ADC1_2};
constexpr IR right_ir = {18, ADC_PINS::ADC1_3};
constexpr IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
constexpr IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};

constexpr std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};

void setup() {
    Serial.begin(115200);
    for (const auto &ir : ir_array)
        ir.setup();

    Serial.println("Test");
}

void loop() {
    // for (const auto &[trig_pin, echo_pin] : ir_array) {h
    //     digitalWrite(trig_pin, HIGH);
    //     delay(100);
    //     Serial.println(analogRead(echo_pin));
    //     digitalWrite(trig_pin, LOW);
    // }
    for (const auto &[trig_pin, echo_pin] : ir_array) {
        u_long a = micros();
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(5);
        int val = analogRead();
        digitalWrite(trig_pin, LOW);
        u_long b = micros();
        delay(100);
        Serial.print(String(val)+" ");
    }
    Serial.println();


    // digitalWrite(front_right_ir.trig_pin, HIGH);
    // delay(10);
    // int true_value = analogRead(front_right_ir.echo_pin);
    // digitalWrite(front_right_ir.trig_pin, LOW);
    // delay(10);
    // digitalWrite(front_right_ir.trig_pin, HIGH);
    // u_long init_time = micros();
    // while (abs(analogRead(front_right_ir.echo_pin) - true_value) >= 10){}
    //
    // u_long final_time = micros();
    // Serial.println(String(final_time - init_time));
    // delay(1000);
    // static int ir_output[6] = {};
    //
    // for (uint8_t i = 0; i < 6; i++) {
    //     digitalWrite(ir_array [0].trig_pin, HIGH);
    //     delayMicroseconds(final_time-init_time);
    //     ir_output[i] = analogRead(ir_array [i].trig_pin);
    //     digitalWrite(ir_array [i].trig_pin, LOW);
    // }
    // Serial.printf("%d",final_time - init_time);

}
