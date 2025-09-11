#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Motor.h"
#include "IR.h"
#include "BNO055.h"

#include "Wire.h"

constexpr IR front_left_ir = {36, ADC_PINS::ADC1_0};
constexpr IR front_right_ir = {33, ADC_PINS::ADC1_5};
constexpr IR left_ir = {35, ADC_PINS::ADC1_2};
constexpr IR right_ir = {18, ADC_PINS::ADC1_3};
constexpr IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
constexpr IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};
constexpr std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};


auto bno = imu(16,21, I2C_NUM_1);



void setup() {
    // Serial.begin(115200);
    // delay(2000);
    // for (const auto &ir : ir_array)
    //     ir.setup();
    // auto hi = bno.mag();
    bno.setup();
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(1000);
}

void loop() {
    auto vec = bno.euler();
    static auto x_init = vec.x();
    float angle = fmod(vec.x() + 180 - x_init, 360.0f);
    if (angle < 0) angle += 360;
    angle -= 180;
    angle *= -1;
    Serial.println(angle);
    delay(100);
}
