#include <Arduino.h>
#include "pins_arduino.h"
#include <esp_adc/adc_oneshot.h>

constexpr uint8_t ADC1_0 = 1;
constexpr uint8_t ADC1_1 = 2;
constexpr uint8_t ADC1_2 = 3;
constexpr uint8_t ADC1_3 = 4;
constexpr uint8_t ADC1_4 = 5;
constexpr uint8_t ADC1_5 = 6;


struct IR {
    uint8_t trig_pin;
    uint8_t echo_pin;
};

IR front_left_ir = {36, ADC1_0};
IR front_right_ir = {33, ADC1_5};
IR left_ir = {35, ADC1_2};
IR right_ir = {18, ADC1_3};
IR left_diagonal_ir = {40, ADC1_1};
IR right_diagonal_ir = {34, ADC1_4};

std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};


void setup() {
    Serial.begin(115200);
    for (const auto &[trig_pin, echo_pin] : ir_array) {
        pinMode(trig_pin, OUTPUT);
        digitalWrite(trig_pin, LOW);
        pinMode(echo_pin, INPUT);
    }
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
        int val = analogRead(echo_pin);
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
