#include <Arduino.h>

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "IR.h"
#include "Motor.h"
#include "Wire.h"


constexpr IR front_left_ir = {36, ADC_PINS::ADC1_0};
constexpr IR front_right_ir = {33, ADC_PINS::ADC1_5};
constexpr IR left_ir = {35, ADC_PINS::ADC1_2};
constexpr IR right_ir = {18, ADC_PINS::ADC1_3};
constexpr IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
constexpr IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};
constexpr std::array ir_array = {front_left_ir,
                                 front_right_ir,
                                 left_ir,
                                 right_ir,
                                 left_diagonal_ir,
                                 right_diagonal_ir};

const auto right_motor = Motor(MOTOR_PINS::IN1, MOTOR_PINS::IN2);
const auto left_motor = Motor(MOTOR_PINS::IN4, MOTOR_PINS::IN3);


// set the default axis
// signs corresponds to default axis
constexpr auto bno_axis_config = Axis_remap_config{
    .x_axis = remap_axis::z_axis,
    .y_axis = remap_axis::y_axis,
    .z_axis = remap_axis::x_axis,

    .x_sign = remap_sign::positive,
    .y_sign = remap_sign::positive,
    .z_sign = remap_sign::negative,
};

auto bno = imu(16, 21, I2C_NUM_0);

float normalize_angle(const float angle, const float init_angle) {
    float ret = fmod(angle + 180 - init_angle, 360.0f);
    if (ret < 0)
        ret += 360;
    ret -= 180;
    return ret * -1;
}

void setup() {
    bno.setup();
    bno.remap_axis(bno_axis_config);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
    delay(1000);
}

void loop() {
    auto vec = bno.euler();
    static auto z_init = vec.z();
    float angle = fmod(vec.z() + 180 - z_init, 360.0f);
    if (angle < 0)
        angle += 360;
    angle -= 180;
    angle *= -1;
    Serial.println(angle);
    delay(100);
}
