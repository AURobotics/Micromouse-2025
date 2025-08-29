#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define left_dir1 22
#define left_dir2 23
#define left_pwm 33

#define right_dir1 25
#define right_dir2 26
#define right_pwm 32

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  pinMode(left_dir1, OUTPUT);
  pinMode(left_dir2, OUTPUT);
  pinMode(left_pwm, OUTPUT);

  pinMode(right_dir1, OUTPUT);
  pinMode(right_dir2, OUTPUT);
  pinMode(right_pwm, OUTPUT);
}

void loop(void)
{
  float heading = getOrientationX();
  Serial.print("Heading (X): ");
  Serial.print(heading);
  Serial.println("°");

  delay(100);
}

float getOrientationX()
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x;
}


/*moveMotors(HIGH, 100, HIGH, 100); // forward
  moveMotors(HIGH, 100, LOW, 100); // tight right
  moveMotors(LOW, 100, HIGH, 100); // tight left
  moveMotors(HIGH, 100, HIGH, 0); // wide right
  moveMotors(HIGH, 0, HIGH, 100); // wide left
  moveMotors(LOW, 100, LOW, 100); // backward
  moveMotors(HIGH, 0, HIGH, 0); // stop*/
void moveMotors(bool dir_left, char pwm_left, bool dir_right, char pwm_right) {
  digitalWrite(left_dir1, dir_left);
  digitalWrite(left_dir2, !dir_left);
  analogWrite(left_pwm, pwm_left);

  digitalWrite(right_dir1, dir_right);
  digitalWrite(right_dir2, !dir_right);
  analogWrite(right_pwm, pwm_right);
}

void rotate(float destination){ //rotates robot to destination
  float difference;
  float source=getOrientationX();
  while(source!=destination){
    source=getOrientationX();
    if(source>destination){
      difference=source-destination;
      if(difference>180){
        moveMotors(HIGH, 100, LOW, 100); // tight right
      }
      else{
        moveMotors(LOW, 100, HIGH, 100); // tight left
      }
    }
    else{
      difference=destination-source;
      if(difference>180){
        moveMotors(LOW, 100, HIGH, 100); // tight left
      }
      else{
        moveMotors(HIGH, 100, LOW, 100); // tight right
      }
    }
  }
}
