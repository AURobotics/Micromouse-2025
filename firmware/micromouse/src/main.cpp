#include <Arduino.h>

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "IR.h"
#include "Motor.h"
#include "Wire.h"


IR front_left_ir = {36, ADC_PINS::ADC1_0};
IR front_right_ir = {33, ADC_PINS::ADC1_5};
IR left_ir = {35, ADC_PINS::ADC1_2};
IR right_ir = {18, ADC_PINS::ADC1_3};
IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};
std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};

const auto right_motor = Motor(MOTOR_PINS::IN4, MOTOR_PINS::IN3);
const auto left_motor = Motor(MOTOR_PINS::IN1, MOTOR_PINS::IN2);

// set the default axis
// signs corresponds to default axis
// constexpr auto bno_axis_config = Axis_remap_config{
//     .x_axis = remap_axis::z_axis,
//     .y_axis = remap_axis::y_axis,
//     .z_axis = remap_axis::x_axis,
//
//     .x_sign = remap_sign::positive,
//     .y_sign = remap_sign::positive,
//     .z_sign = remap_sign::negative,
// };


[[noreturn]]
void IR_task(void* pvParameters) {
    uint8_t i = 0;
    for (;;) {
        Serial.print(String(ir_array[i].read()) + String(" "));
        i = (i + 1) % 6;
        if (i == 5)
            Serial.println("");
        vTaskDelay(5);
    }
}

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    // clang-format off
    xTaskCreate(IR_task,
        "IR_task",
        2048,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr);
    // clang-format on
}

void loop() {
    vTaskDelete(nullptr);
}
