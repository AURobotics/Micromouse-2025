#pragma once
#include <Wire.h>
#include <driver/i2c.h>
#include "Arduino.h"
#include "BNO055_registers.h"

/*
TODO: pass wire object as parameter with default value. On esp, start it on
TODO: I2C port 0.
TODO: add esp-idf compatability
TODO: IMU zeroing
*/

struct Calibration_t {
  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
};

struct Axis_remap_config {
  remap_axis x_axis;
  remap_axis y_axis;
  remap_axis z_axis;

  remap_sign x_sign;
  remap_sign y_sign;
  remap_sign z_sign;
};

struct vec_3 {
  float vec[3];
  float& x() { return vec[0]; }
  float& y() { return vec[1]; }
  float& z() { return vec[2]; }
  const float& x() const { return vec[0]; }
  const float& y() const { return vec[1]; }
  const float& z() const { return vec[2]; }
};

struct vec_4 {
  float vec[4];
  float& w() { return vec[0]; }
  float& y() { return vec[1]; }
  float& z() { return vec[2]; }
  float& x() { return vec[3]; }
  const float& w() const { return vec[0]; }
  const float& y() const { return vec[1]; }
  const float& z() const { return vec[2]; }
  const float& x() const { return vec[3]; }
};


struct imu {
public:
  imu(int8_t SCL, int8_t SDA, i2c_port_t port = I2C_NUM_0,
      uint8_t address = 0x29);void setup()const;
  void set_mode(operation_mode mode) const;
  void remap_axis(uint8_t config) const; // 0 to 7
void remap_axis(Axis_remap_config config)const;
  void remap_axis(enum remap_axis axis, remap_sign) const;
  void calibration_status(Calibration_t& s) const;

  // void set_units_metric() const;
  // void set_units_default() const;

  int8_t temperature() const;
  vec_3 acceleration() const;
  vec_3 gyro() const;
  vec_3 mag() const;
  vec_3 euler() const;
  vec_4 quaternion() const;


private:
  int8_t SCL;
  int8_t SDA;
  uint8_t address = 0X29;
  TwoWire& i2c;

  void write_register(uint8_t reg, uint8_t val) const;
  uint8_t read_register(uint8_t reg, uint8_t* buffer, uint8_t len = 8) const;
};