#include "Arduino.h"
#include "esp32-hal-timer.h"
#include "motor_driver.h"

#define TICKS_PER_REV 1400
#define WHEEL_RADIUS 0.034 //in m
#define WHEEL_CIRCUMFERENCE (2 * M_PI * WHEEL_RADIUS)

hw_timer_t *Timer0_Cfg = nullptr;

volatile int sample_count = 0;
volatile bool first_time = true;

auto right_motor = Motor(motor_pins_t{10, 9, 17, 15});
auto left_motor = Motor(motor_pins_t{7, 8, 13, 14});

void IRAM_ATTR Timer0_ISR() {
  if (sample_count == 200) {
    first_time = false;
    left_motor.m_encoder.clearCount();
    right_motor.m_encoder.clearCount();

    left_motor.move(Direction::CW, 1024);
    right_motor.move(Direction::CW, 1024);
  }
  if (sample_count >= 5200) {
    left_motor.move(Direction::CW, 0);
    right_motor.move(Direction::CW, 0);

    timerEnd(Timer0_Cfg);
    return;
  }

  const auto lr = static_cast<double>(left_motor.get_count()) / TICKS_PER_REV *
                  WHEEL_CIRCUMFERENCE;
  const auto lf = static_cast<double>(right_motor.get_count()) / TICKS_PER_REV *
                  WHEEL_CIRCUMFERENCE;

  Serial.printf("%llu,%f,%f\n", timerReadMillis(Timer0_Cfg), lr, lf);
  sample_count += 1;
}

void setup() {
  Serial.begin(921600);
  Serial.flush();

  right_motor.setup(Encoder_mode::FULL_QUAD);
  left_motor.setup(Encoder_mode::FULL_QUAD);

  delay(5000);
  Serial.println("hi");
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 1000, true, 5000);
}

void loop() { vTaskDelete(nullptr); }
// using pio, run "pio device monitor -b 921600 > file.txt"