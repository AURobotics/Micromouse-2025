#include "BNO055.h"
#include "Arduino.h"
#include "BNO055_registers.h"

imu::imu(const int8_t SCL, const int8_t SDA, const i2c_port_t port,
         const uint8_t address) :
    SCL(SCL), SDA(SDA), address(address), i2c(port == 0 ? Wire : Wire1) {}


void imu::setup(const bool set_x_offset) {
    while (true) {
        if (i2c.begin(SDA, SCL))
            break;
        vTaskDelay(850);
    }
    i2c.setClock(400000);
    this->set_mode(operation_mode::CONFIG);
    vTaskDelay(20);
    // celsius, radians, rad/s, m/s2
    constexpr uint8_t unit = 1 << 2 | 1 << 1;
    write_register(imu_registers::unit::UNIT_SEL, unit);
    set_mode(operation_mode::NDOF);
    vTaskDelay(20);
    if (set_x_offset)
        set_yaw_offset();
}

void imu::set_mode(operation_mode mode) const {
    write_register(imu_registers::mode::OPR_MODE, static_cast<uint8_t>(mode));
}

void imu::remap_axis(const uint8_t config) const {
    this->set_mode(operation_mode::CONFIG);
    if (config > 7)
        return;
    remap_axis(static_cast<::remap_axis>(axis_remap_table[config].first),
               static_cast<remap_sign>(axis_remap_table[config].second));
    this->set_mode(operation_mode::NDOF);
}


void imu::remap_axis(Axis_remap_config config) const {
    this->set_mode(operation_mode::CONFIG);
    const auto config_axis = static_cast<uint8_t>(config.x_axis) |
        static_cast<uint8_t>(config.y_axis) << 2 |
        static_cast<uint8_t>(config.z_axis) << 4;
    write_register(imu_registers::axis_remap::REMAP_AXIS, config_axis);

    const auto config_sign = static_cast<uint8_t>(config.x_sign) << 2 |
        static_cast<uint8_t>(config.y_sign) << 1 |
        static_cast<uint8_t>(config.z_sign);
    write_register(imu_registers::axis_remap::REMAP_SIGN, config_sign);
    this->set_mode(operation_mode::NDOF);
}

void imu::calibration_status(Calibration_t& s) const {
    uint8_t calib = 0;
    read_register(imu_registers::status::CALIB, &calib, 1);
    s.sys = calib >> 6 & 0x03;
    s.gyro = calib >> 4 & 0x03;
    s.accel = calib >> 2 & 0x03;
    s.mag = calib >> 0 & 0x03;
}

// void imu::set_units_metric() const {
//     write_register(imu_registers::unit::UNIT_SEL, 0x00);
// }
//
// void imu::set_units_default() const {
//     write_register(imu_registers::unit::UNIT_SEL, 0x80);
// }

int8_t imu::temperature() const {
    uint8_t t;
    read_register(imu_registers::sensor_data::TEMP, &t, 1);
    return static_cast<int8_t>(t);
}

vec_3 imu::acceleration() const {
    vec_3 v{};
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::ACCEL_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i) {
        v.vec[i] = static_cast<float>(buf[2 * i + 1] << 8 | buf[2 * i]) /
            100.0f; // scale: 1 LSB = 1 mg = 0.01 m/s^2
    }
    return v;
}

vec_3 imu::gyro() const {
    vec_3 v{};
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::GYRO_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] =
            static_cast<float>(buf[2 * i + 1] << 8 | buf[2 * i]) / 900.0f;
    return v;
}

vec_3 imu::mag() const {
    vec_3 v{};
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::MAG_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] = static_cast<float>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16.0f;
    return v;
}

/**
 * x, y, z
 */
vec_3 imu::euler() const {
    vec_3 v{};
    uint8_t buf[6] = {};
    read_register(imu_registers::sensor_data::EUL_X_LSB, buf, 6);
    for (int i = 0; i < 3; ++i)
        v.vec[i] = static_cast<float>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16.0f;
    return v;
}

// set only for micromouse
void imu::set_yaw_offset() { this->yaw_offset = this->euler().x(); }

float imu::relative_heading() const {
    uint8_t buf[2] = {};
    read_register(imu_registers::sensor_data::EUL_X_LSB, buf, 2);
    float angle = static_cast<float>(buf[1] << 8 | buf[0]) / 16.0f;
    angle = fmod(angle + 180 - this->yaw_offset, 360.0f);
    if (angle < 0)
        angle += 360;
    angle -= 180;
    angle *= -1;
    return angle;
}

/**
 * w, x, y, z
 */
vec_4 imu::quaternion() const {
    vec_4 v{};
    using imu_registers::sensor_data;
    uint8_t buf[8];
    read_register(sensor_data::QUAT_W_LSB, buf, 8);
    for (int i = 0; i < 4; ++i)
        v.vec[i] =
            static_cast<float>(buf[2 * i + 1] << 8 | buf[2 * i]) / 16384.0f;
    return v;
}


void imu::write_register(const uint8_t reg, const uint8_t val) const {
    i2c.beginTransmission(address);
    i2c.write(reg);
    i2c.write(val);
    i2c.endTransmission();
}

uint8_t imu::read_register(const uint8_t reg, uint8_t* buffer,
                           const uint8_t len) const {
    i2c.beginTransmission(address);
    i2c.write(reg);
    i2c.endTransmission(false);
    i2c.requestFrom(address, len);

    uint8_t count = 0;
    for (uint8_t i = 0; i < len && i2c.available(); ++i) {
        buffer[i] = i2c.read();
        ++count;
    }
    return count;
}
