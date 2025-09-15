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
// constexpr auto bno_axis_config = Axis_remap_config{
//     .x_axis = remap_axis::z_axis,
//     .y_axis = remap_axis::y_axis,
//     .z_axis = remap_axis::x_axis,
//
//     .x_sign = remap_sign::positive,
//     .y_sign = remap_sign::positive,
//     .z_sign = remap_sign::negative,
// };

auto bno = imu(16, 21, I2C_NUM_0);

float normalize_angle(const float angle, const float init_angle) {
    float ret = fmod(angle + 180 - init_angle, 360.0f);
    if (ret < 0)
        ret += 360;
    ret -= 180;
    return ret * -1;
}


void setup() {
    Serial.begin(115200);
    bno.setup(true);
    // bno.remap_axis(bno_axis_config);
    pcnt_unit_handle_t pcnt_handler = nullptr;
    pcnt_unit_config_t unit_config = {
        .low_limit = INT16_MIN,
        .high_limit = INT16_MAX
    };
    unit_config.flags.accum_count = 1;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_handler));

    constexpr pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = 2,
        .level_gpio_num = 4,
    };

    pcnt_channel_handle_t chan_a_handle = nullptr;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_handler, &chan_a_config, &chan_a_handle));

    constexpr pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = 4,
        .level_gpio_num = 2,
    };

    pcnt_channel_handle_t chan_b_handle = nullptr;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_handler, &chan_b_config, &chan_b_handle));

    gpio_pullup_en(GPIO_NUM_0);
    gpio_pullup_en(GPIO_NUM_1);

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(chan_a_handle,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(chan_a_handle,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(chan_b_handle,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(chan_b_handle,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_handler));
}

void loop() {
    Serial.println();
    delay(100);
}
