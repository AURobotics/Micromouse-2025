#pragma once
#include "constants.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

/**
 * \enum Encoder_mode
 * \brief Defines the encoder decoding modes.
 * - FULL_QUAD: Accounts for Rising and falling edges of both encoder phases.
 * - HALF_QUAD: Accounts for only the Rising edges.
 * - SINGLE: Accounts for only the rising edge of one phase.
 */
enum class Encoder_mode { FULL_QUAD = 4, HALF_QUAD = 2, SINGLE = 1 };

/**
 * @enum Direction
 */
enum class Direction { CW, CCW };

/**
 * @brief holds pid constants for motor driver
 * @struct pid_const
 */

/**
 * \brief Represents the pin configuration for a motor.
 */
struct motor_pins_t {
  int8_t pA{};   ///< Pin A for encoder phase A.
  int8_t pB{};   ///< Pin B for the encoder phase B.
  int8_t dir1{}; ///< Direction control pin 1.
  int8_t dir2{}; ///< Direction control pin 2.
};

class Motor {
public:
  ~Motor() = default;
  /**
   *
   * @param pins pins of motor
   * @param encoder_mode encoder Mode, each mode corresponds to different
   * decoding of the encoder signal
   */
  explicit Motor(const motor_pins_t pins,
                 const Encoder_mode encoder_mode = Encoder_mode::FULL_QUAD)
      : m_direction_pin_1(pins.dir1), m_direction_pin_2(pins.dir2) {

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    switch (encoder_mode) {
    case Encoder_mode::FULL_QUAD: {
      m_encoder.attachFullQuad(pins.pA, pins.pB);
      break;
    }
    case Encoder_mode::HALF_QUAD: {
      m_encoder.attachHalfQuad(pins.pA, pins.pB);
      break;
    }
    case Encoder_mode::SINGLE: {
      m_encoder.attachSingleEdge(pins.pA, pins.pB);
      break;
    }
    }
    m_encoder.clearCount();

    analogWriteResolution(m_direction_pin_1, 10);
    analogWriteResolution(m_direction_pin_2, 10);
    pinMode(m_direction_pin_1, OUTPUT);
    pinMode(m_direction_pin_2, OUTPUT);
    analogWrite(m_direction_pin_1, 0);
    analogWrite(m_direction_pin_2, 0);
  }

  /**
   * @return rpm of object Motor
   */
  double rpm();

  /**
   * @return absolute count of encoder ticks
   */
  int64_t get_count() { return m_encoder.getCount(); }

  /**
   * moves the motors on one side
   * @param direction
   * @param duty
   */
  void move(Direction direction, uint16_t duty) const;

  /**
   * computes the motor driver input using pid controller
   * @param target rpm setpoint
   * @param current current rpm, retrieved from rpm() function
   * @return rpm value to be mapped to duty cycle
   */
  double compute_rpm(double target, double current);

  /**
   * @brief holds number of ledc channels acquired
   */
  static inline uint8_t ledc_channel_count = 0;

  ESP32Encoder m_encoder;

private:
  // ReSharper disable once CppDFANotInitializedField
  int8_t m_direction_pin_1;
  int8_t m_direction_pin_2;
  int64_t rpm_prev_pulses = 0;
  unsigned long rpm_prev_time = micros();
  double prev_rpm = 0;

  Direction m_prev_direction = Direction::CW;
  double accum = 0;
  double prev_err = 0;
  double prev_time = micros();

  double m_output = 0;

  uint8_t m_ledc_channel_1{};
  uint8_t m_ledc_channel_2{};
};