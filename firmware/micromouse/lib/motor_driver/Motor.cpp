#include "Motor.h"


Motor::Motor(const MOTOR_PINS in1, const MOTOR_PINS in2, const ENC_PINS encA, const ENC_PINS encB) :
    IN1(in1), IN2(in2), ENC_A(encA), ENC_B(encB) {
    pinMode(static_cast<uint8_t>(IN1), OUTPUT);
    pinMode(static_cast<uint8_t>(IN2), OUTPUT);
    pinMode(static_cast<uint8_t>(ENC_A), INPUT_PULLUP);
    pinMode(static_cast<uint8_t>(ENC_B), INPUT_PULLUP);

    analogWriteResolution(static_cast<uint8_t>(this->IN1), 10);
    analogWriteResolution(static_cast<uint8_t>(this->IN2), 10);
    this->move(0);


    pcnt_unit_config_t unit_config = {.low_limit = INT16_MIN, .high_limit = INT16_MAX};
    unit_config.flags.accum_count = 1;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m_pcnt_handler));

    pcnt_chan_config_t chan_a_config = { //make constexpr
        .edge_gpio_num = static_cast<uint8_t>(this->ENC_A),
        .level_gpio_num = static_cast<uint8_t>(this->ENC_B),
    };

    pcnt_channel_handle_t chan_a_handle = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(m_pcnt_handler, &chan_a_config, &chan_a_handle));

    pcnt_chan_config_t chan_b_config = { //make constexpr
        .edge_gpio_num = static_cast<uint8_t>(this->ENC_A),
        .level_gpio_num = static_cast<uint8_t>(this->ENC_B),
    };

    pcnt_channel_handle_t chan_b_handle = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(m_pcnt_handler, &chan_b_config, &chan_b_handle));


    //supposedly this setup is correct
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    ESP_ERROR_CHECK(pcnt_unit_enable(m_pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_start(m_pcnt_handler));

}

// duty 0 to 1024, STOP sets both outputs to HIGH
void Motor::move(const uint16_t duty, const Direction direction) const {
    if (direction == Direction::FORWARD || direction == Direction::BACKWARD) {
        analogWrite(static_cast<uint8_t>(this->IN1), duty * static_cast<uint8_t>(direction));
        analogWrite(static_cast<uint8_t>(this->IN2), duty * !static_cast<uint8_t>(direction));
    }
    else {
        analogWrite(static_cast<uint8_t>(this->IN1), 1024);
        analogWrite(static_cast<uint8_t>(this->IN2), 1024);
    }

}
