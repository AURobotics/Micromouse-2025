#include <Arduino.h>

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "BNO055.h"
#include "IR.h"
#include "Motor.h"
#include "Wire.h"
// #include "ESP32Encoder.h"
#include "RotaryEncoderPCNT.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>


#define ticksperlafa 1400
#define circumference 10.681
#define distance_between_wheels 9
// #define PI 3.141592653589


inline void getPosition(); // odom
inline float getOrientationX();
inline float getRate();
void moveF(double tiles);
void turn(double angle);
double fixSpeed(double speed);


constexpr IR front_left_ir = {36, ADC_PINS::ADC1_0};
constexpr IR front_right_ir = {33, ADC_PINS::ADC1_5};
constexpr IR left_ir = {35, ADC_PINS::ADC1_2};
constexpr IR right_ir = {18, ADC_PINS::ADC1_3};
constexpr IR left_diagonal_ir = {40, ADC_PINS::ADC1_1};
constexpr IR right_diagonal_ir = {34, ADC_PINS::ADC1_4};
constexpr std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};

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

// auto bno = imu(16, 21, I2C_NUM_0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

int readings[6];

float normalize_angle(const float angle, const float init_angle) {
    float ret = fmod(angle + 180 - init_angle, 360.0f);
    if (ret < 0)
        ret += 360;
    ret -= 180;
    return ret * -1;
}

pcnt_unit_handle_t pcnt_handler = nullptr;


RotaryEncoderPCNT rightEncoder(7, 8);
RotaryEncoderPCNT leftEncoder(10, 9);
double previousRight;
double previousLeft;

int left_revolutions, prev_left_revolutions;
int right_revolutions, prev_right_revolutions;


double xPosition = 0, yPosition = 0;
double yaw = 0;
int theoreticalHeading = 0;


void setup() {
    Serial.begin(115200);
    // bno.setup(true);
    Wire.begin(21, 16);
    bno = Adafruit_BNO055(55, 0x29, &Wire);
    if (!bno.begin()) // lol
    {
        // Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    // delay(1000);
    bno.setExtCrystalUse(true);


    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    // bno.remap_axis(bno_axis_config);
    pcnt_unit_config_t unit_config = {.low_limit = INT16_MIN, .high_limit = INT16_MAX};
    unit_config.flags.accum_count = 1;

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_handler));

    constexpr pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = static_cast<uint8_t>(ENC_PINS::PA_1),
        .level_gpio_num = static_cast<uint8_t>(ENC_PINS::PA_1),
    };

    pcnt_channel_handle_t chan_a_handle = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_handler, &chan_a_config, &chan_a_handle));

    constexpr pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = 4,
        .level_gpio_num = 2,
    };

    pcnt_channel_handle_t chan_b_handle = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_handler, &chan_b_config, &chan_b_handle));

    gpio_pullup_en(GPIO_NUM_0);
    gpio_pullup_en(GPIO_NUM_1);

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_b_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_handler));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_handler));

    delay(2000); // don't delete this , the bno doesn't work at first
}

void loop() {
    int counter = 0;
    pcnt_unit_get_count(pcnt_handler, &counter);
    Serial.println(counter);
    // Serial.println(counterbno.relative_heading());
    // delay(100);
    // right_motor.move(100,static_cast<Direction>(1));
    //  Serial.print(leftEncoder.position());
    //  Serial.print(" ");
    //  Serial.println(rightEncoder.position());
    // Serial.println(bno.gyro().x());
    //  getPosition();
    //  Serial.print(xPosition);
    //  Serial.print(" ");
    //  Serial.print(yPosition);
    //  Serial.print(" ");
    //  Serial.println(yaw);
    //  delay(100);
    // turn(-90);
    //  moveF(1);
    //  delay(2000);
    //  Serial.println(yaw);
}


double fixSpeed(double speed) {
    int maxx = 100;
    speed = constrain(speed, -maxx, maxx);
    if (abs(speed) < 5)
        return 0;
    if (speed > 0)
        return map(speed, 0, maxx, 45, maxx);
    if (speed < 0)
        return map(speed, -maxx, 0, -maxx, -45);
    return 0;
}

inline double calculateDistance(double x, double y) { return sqrt(pow(x - xPosition, 2) + pow(y - yPosition, 2)); }

double angleDiff(double start, double goal) {
    // goal  = (goal + 360) % 360.0;
    // start += 360;
    // goal += 360;
    goal += (goal > 360 ? -360 : 0);
    goal += (goal < 0 ? 360 : 0);
    double angle = goal - start;
    double angle2 = angle + (angle < 0 ? 360 : -360);
    return (abs(angle) < abs(angle2) ? angle : angle2);
}


inline float getOrientationX() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    return orientationData.orientation.x;
}

inline float getRate() {
    sensors_event_t gyroData;
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyroData.gyro.x; // might change it to gyro.z msh x , haven't tested yet
                            // -----------------------------------------------------------------------IMPORTANT
}

inline float getLin() {
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    return linearAccelData.acceleration.z;
}

inline void getPosition() {
    double leftRevolutions = 1.0 * leftEncoder.position() / ticksperlafa;
    double rightRevolutions = 1.0 * rightEncoder.position() / ticksperlafa;

    double leftDistance = (leftRevolutions - previousLeft) * circumference;
    double rightDistance = (rightRevolutions - previousRight) * circumference;
    double distance = (leftDistance + rightDistance) / 2;

    double encoderDelta =
        (rightDistance - leftDistance) / distance_between_wheels; // won't use that , and this is in radian
    // double bnoDelta = (bno.relative_heading() - yaw);
    double bnoDelta = (getOrientationX() - yaw);
    double deltaAngle = bnoDelta; // add the encoder delta to it if you want

    yPosition += distance * cos((yaw + deltaAngle / 2) * PI / 180);
    xPosition += distance * sin((yaw + deltaAngle / 2) * PI / 180);
    yaw += deltaAngle; // or yaw = getOrientationX();

    // yaw = (yaw+360)%360;

    previousLeft = leftRevolutions;
    previousRight = rightRevolutions;
}

// 2 -5
void turn(double angle) {
    getPosition();
    double desiredAngle = yaw + angle;
    double error = angleDiff(yaw, desiredAngle);
    bool direction = (error > 0 ? true : false); // true -> turn right | false -> turn left
    double errorPrev = error;
    double totalerror = 0;
    unsigned long t = millis();

    double kp = 2; // Kp and Kd will be set with testing
    double kd = -5;

    double speed = 100;

    // while(abs(error) > 2|| fabs(bno.gyro().x()) > 1)
    while (abs(error) > 2 || fabs(getRate()) > 1) {
        getPosition();
        error = angleDiff(yaw, desiredAngle);

        Serial.print(desiredAngle);
        Serial.print(" ");
        Serial.print(yaw);
        Serial.print(" ");
        Serial.println(error);
        // speed = kp * error + kd * bno.gyro().x();
        speed = kp * error + kd * getRate();
        speed = fixSpeed(speed);
        // //Serial.print(" ");
        // //Serial.print(speed);
        // //Serial.print(" ");
        // //Serial.println(getRate());
        direction = (speed > 0 ? true : false);
        left_motor.move(abs(speed), static_cast<Direction>(!direction));
        right_motor.move(abs(speed), static_cast<Direction>(direction));
        // analogWrite(leftMotorForward , (direction)  * abs(speed) );
        // analogWrite(leftMotorBackward , (!direction) * abs(speed));

        // analogWrite(rightMotorForward , (!direction) * abs(speed));
        // analogWrite(rightMotorBackward , (direction)  * abs(speed));


        errorPrev = error;
        totalerror += error;
        t = millis();
    }
    getPosition();
    // Serial.println(yaw);
    //   analogWrite(leftMotorForward ,   0);
    //   analogWrite(leftMotorBackward ,  0);
    //   analogWrite(rightMotorForward ,  0);
    //   analogWrite(rightMotorBackward , 0);
    left_motor.move(0, Direction::STOP);
    right_motor.move(0, Direction::STOP);
}


// 2 -1
//-1.2 1.5
void moveF(double tiles = 16) // if you want to move tile by tile use moveF(1), if you want continuous use moveF();
{ // just need to add to make it stop using the irs
    double desiredDistance = tiles *
        20; // el tile el mafrood 18cm, bas we found it would move slightly less than what we wanted, fa we increased it

    double startX = xPosition, startY = yPosition;
    double startYaw = theoreticalHeading;

    unsigned long startRight = rightEncoder.position();
    unsigned long startLeft = leftEncoder.position();
    unsigned long rightTicks, leftTicks;
    unsigned long startTime = millis();

    // for the distance
    double errorL = desiredDistance - calculateDistance(startX, startY);
    double errorLPrev = errorL;

    bool direction = (errorL >= 0 ? true : false); // true -> forward, false -> backward

    // to keep moving staight
    double errorA = angleDiff(yaw, startYaw);
    double errorAPrev = errorA;

    unsigned long t = millis();

    double Kpl = 2; // KD AND KP are changed with testing
    double Kdl = -1;

    double Kpa = -1.2;
    double Kda = 1.5;

    double speedl;
    double speeda;
    double speed;
    ////Serial.println(errorL);

    // unsigned long  timeout_timer = millis();
    char timeout_ctr = 0;

    while ((abs(errorL) > 0.2) && timeout_ctr < 50) // this 1 might change
    {
        // READIRS();
        rightTicks = rightEncoder.position() - startRight;
        leftTicks = leftEncoder.position() - startLeft;
        unsigned long deltaTicks = leftTicks - rightTicks;

        getPosition();
        errorL = desiredDistance - calculateDistance(startX, startY);
        errorA = angleDiff(yaw, startYaw);


        speedl = Kpl * errorL + Kdl * (errorL - errorLPrev) / (millis() - t);
        speeda = Kpa * errorA + Kda * getRate();
        direction = (speedl >= 0 ? true : false);

        // Serial.print(errorL);
        // Serial.print(" ");
        Serial.print(yaw);
        Serial.print(" ");
        // Serial.print(speedl);
        Serial.print(fixSpeed(speedl - speeda));
        Serial.print(" ");
        // Serial.println(speeda);
        Serial.print(fixSpeed(speedl + speeda));

        Serial.print(" ");

        // analogWrite(leftMotorForward , direction * abs(fixSpeed(speedl - speeda  + (readings[2] > 1200)*20 ))); //
        // might change the +- signs here analogWrite(leftMotorBackward , (!direction) * abs(fixSpeed(speedl - speeda +
        // (readings[2] > 1200)*20 )));
        left_motor.move(abs(fixSpeed(speedl - speeda + (readings[2] > 1200) * 20)), static_cast<Direction>(!direction));


        // analogWrite(rightMotorForward , direction * abs(fixSpeed(speedl + speeda + (readings[3] > 1200)*20 ) ));
        // analogWrite(rightMotorBackward , (!direction) * abs(fixSpeed(speedl + speeda + (readings[3] > 1200)*20) ));
        right_motor.move(abs(fixSpeed(speedl + speeda + (readings[3] > 1200) * 20)),
                         static_cast<Direction>(!direction));

        errorLPrev = errorL;
        errorAPrev = errorA;
        t = millis();
        // if(t - startTime > 5000)
        if (fabs(getLin()) < 0.1) {
            // timeout_timer = millis();
            timeout_ctr++;
        }
        Serial.println(getLin());
    }

    Serial.print("Done ");
    //   Serial.println(calculateDistance(startX,startY));
    Serial.println(errorL);
    //   analogWrite(leftMotorForward ,   0);
    //   analogWrite(leftMotorBackward ,  0);
    //   analogWrite(rightMotorForward ,  0);
    //   analogWrite(rightMotorBackward , 0);
    right_motor.move(0, Direction::STOP);
    left_motor.move(0, Direction::STOP);
}
