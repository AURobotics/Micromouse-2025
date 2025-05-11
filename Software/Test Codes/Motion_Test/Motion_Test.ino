// https://www.upesy.com/blogs/tutorials/esp32-pinout-reference-gpio-pins-ultimate-guide#

#define left_dir1 22
#define left_dir2 23
#define left_pwm 33

#define right_dir1 25
#define right_dir2 26
#define right_pwm 32

void setup() {
  // put your setup code here, to run once:

  pinMode(left_dir1, OUTPUT);
  pinMode(left_dir2, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_dir1, OUTPUT);
  pinMode(right_dir2, OUTPUT);
  pinMode(right_pwm, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  moveMotors(HIGH, 100, HIGH, 100); // forward
  delay(1000);
  moveMotors(HIGH, 100, LOW, 100); // tight right
  delay(1000);
  moveMotors(LOW, 100, HIGH, 100); // tight left
  delay(1000);
  moveMotors(HIGH, 100, HIGH, 0); // wide right
  delay(1000);
  moveMotors(HIGH, 0, HIGH, 100); // wide left
  delay(1000);
  moveMotors(LOW, 100, LOW, 100); // backward
  delay(1000);
  moveMotors(HIGH, 0, HIGH, 0); // stop
  delay(1000);

}

void moveMotors(bool dir_left, char pwm_left, bool dir_right, char pwm_right) {
  digitalWrite(left_dir1, dir_left);
  digitalWrite(left_dir2, !dir_left);
  analogWrite(left_pwm, pwm_left);

  digitalWrite(right_dir1, dir_right);
  digitalWrite(right_dir2, !dir_right);
  analogWrite(right_pwm, pwm_right);
}
