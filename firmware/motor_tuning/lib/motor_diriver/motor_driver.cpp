#include "motor_driver.h"
#include <cfloat>
using std::abs;
using std::max;
using std::min;
#define clamp(v, l, h) min(max(v, l), h)

double Motor::rpm()
{
  const auto count = m_encoder.getCount();
  const unsigned long time = micros();
  const double dt = (time - rpm_prev_time) / 1000000.0;
  const int64_t dp = count - rpm_prev_pulses;

  this->rpm_prev_time = time;
  this->rpm_prev_pulses = count;

  if (dt < DBL_EPSILON)
    return prev_rpm;
  prev_rpm = dp / dt * 60 / WHEEL_PPR;
  return prev_rpm;
}

void Motor::move(const Direction direction, const uint16_t duty) const
{
  if (direction == Direction::CW)
  {
    analogWrite(m_direction_pin_1, duty);
    analogWrite(m_direction_pin_2, 0);
  } else
  {
    analogWrite(m_direction_pin_1, 0);
    analogWrite(m_direction_pin_2, duty);
  }
}
