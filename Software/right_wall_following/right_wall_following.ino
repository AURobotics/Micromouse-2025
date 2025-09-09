#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RotaryEncoderPCNT.h>

inline void getPosition(); // odom
inline float getOrientationX();
inline float getRate();
void moveF(double tiles);
void turn(double angle);
inline void FORWARD();

inline void READIRS(); 


#define ticksperlafa 1400
#define circumference 10.681
#define distance_between_wheels  9
#define PI 3.141592653589

#define NUM_IR_SENSORS 6

#define FRONT_RIGHT_IR 0
#define FRONT_LEFT_IR 1
#define DIAGONAL_RIGHT_IR 2
#define DIAGONAL_LEFT_IR 3
#define RIGHT_IR 4
#define LEFT_IR 5

// right motor pins
#define rightMotorForward 13
#define rightMotorBackward 14
RotaryEncoderPCNT rightEncoder(9,8);
double previousRight;

// left motor pins
#define leftMotorForward 17
#define leftMotorBackward 15
RotaryEncoderPCNT leftEncoder(7, 10);
double previousLeft;


int left_revolutions, prev_left_revolutions;
int right_revolutions, prev_right_revolutions;



double xPosition = 0,yPosition = 0;
double yaw = 0;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);



constexpr uint8_t ADC1_0 = 1;
constexpr uint8_t ADC1_1 = 2;
constexpr uint8_t ADC1_2 = 3;
constexpr uint8_t ADC1_3 = 4;
constexpr uint8_t ADC1_4 = 5;
constexpr uint8_t ADC1_5 = 6;


struct IR {
    uint8_t trig_pin;
    uint8_t echo_pin;
};

IR front_left_ir = {36, ADC1_0};
IR front_right_ir = {33, ADC1_5};
IR left_ir = {35, ADC1_2};
IR right_ir = {18, ADC1_3};
IR left_diagonal_ir = {40, ADC1_1};
IR right_diagonal_ir = {34, ADC1_4};

std::array ir_array = {front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir};

int readings[6];


void setup() {

  Serial.begin(115200);

  for (const auto &[trig_pin, echo_pin] : ir_array) {
    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);
    pinMode(echo_pin, INPUT);
  }

  Wire.begin(21,16);
  bno = Adafruit_BNO055(55, 0x29, &Wire);
  while (!Serial) delay(10);
  Serial.println("start");
  if (!bno.begin())    // lol 
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //delay(1000);
  bno.setExtCrystalUse(true);

  analogWrite(leftMotorForward ,   0);
  analogWrite(leftMotorBackward ,  0);
  analogWrite(rightMotorForward ,  0);
  analogWrite(rightMotorBackward , 0);
  leftEncoder.setPosition(0);
  rightEncoder.setPosition(0);
  getPosition();

}

void loop() {
  // put your main code here, to run repeatedly:

  READIRS();
  //-----------------------------------------------------------------------------  didn't rly get it to work, need tinkering with the numbers -----------------------
  if(readings[3] < 700) //right doesn't see wall 
  {
    Serial.println("turning right");
    stop();
    moveF(1.0* 10 / 18); // /18 because moveF moves multiples of tile size(18cm)
    turn(90);
    moveF(1);
  }
  else if(readings[3] > 700 && (readings[1] < 700 || readings[0] < 700)) //  right ir sees wall and front doesn't
  {
    Serial.println("moving forward");
    FORWARD();
  }
  else 
  {
    Serial.println("turning left");
    stop();
    turn(-90);
  }




}

void turn(double angle)
{
  getPosition();
  double desiredAngle = yaw + angle;
  double error = angleDiff(yaw,desiredAngle);
  bool direction = ( error > 0 ? true:false ); // true -> turn right | false -> turn left                      
  double errorPrev = error;
  double totalerror = 0;
  unsigned long t = millis();

  double kp = 1; // Kp and Kd will be set with testing
  double kd = 4;
  double ki = 0;

  double speed = 100;

  while(abs(error) > 3|| fabs(getRate()) > 2.5)
  {
    getPosition();
    error = angleDiff(yaw,desiredAngle);
    
    // Serial.print(desiredAngle);
    // Serial.print(" ");
    // Serial.print(yaw);
    // Serial.print(" ");
    // Serial.print(error);
    speed = kp * error + kd * getRate();
    speed = fixSpeed(speed);
    // Serial.print(" ");
    // Serial.print(speed);
    // Serial.print(" ");
    // Serial.println(getRate());
    direction = ( speed >0 ? true:false );
    analogWrite(leftMotorForward , (direction)  * abs(speed) );
    analogWrite(leftMotorBackward , (!direction) * abs(speed));

    analogWrite(rightMotorForward , (!direction) * abs(speed));
    analogWrite(rightMotorBackward , (direction)  * abs(speed));


    errorPrev = error;
    totalerror += error;
    t = millis();

  }
  getPosition();
  //Serial.println(yaw);
  analogWrite(leftMotorForward ,   0);
  analogWrite(leftMotorBackward ,  0);
  analogWrite(rightMotorForward ,  0);
  analogWrite(rightMotorBackward , 0);


}
//3 -1
//4 -1

inline void FORWARD() // moveF will need some modifications to just work as this one, aw maybe it alr does, haven't tried it rly, just used this one for simplicity 
{
  analogWrite(rightMotorForward, 120);
  analogWrite(rightMotorBackward, 0);
  analogWrite(leftMotorForward, 120);
  analogWrite(leftMotorBackward, 0);
}

void moveF(double tiles = 16)// if you want to move tile by tile use moveF(1), if you want continuous use moveF();
{                            // just need to add to make it stop using the irs            
  double desiredDistance = tiles * 18;

  double startX = xPosition, startY = yPosition;
  double startYaw = yaw;
  


  // for the distance 
  double errorL = desiredDistance - calculateDistance(startX,startY);
  double errorLPrev = errorL;
  
  bool direction = ( errorL >= 0 ? true:false ); // true -> forward, false -> backward
  
  // to keep moving staight 
  double errorA = yaw-startYaw;
  double errorAPrev = errorA;

  unsigned long t = millis();

  double Kpl = 3; // KD AND KP are changed with testing 
  double Kdl = -1;

  double Kpa = 4;
  double Kda = -2;

  double speedl;
  double speeda;
  double speed;
  //Serial.println(errorL);

  while(abs(errorL) > 0.2) // this 1 might change
  {
    READIRS(); 
    if(readings[0] > 700|| readings[1] > 700 || readings[3] < 700) // if the front irs see wall, or the right one doesn't  break;
    {
      break;
    }
    getPosition();
    errorL = desiredDistance - calculateDistance(startX,startY);
    errorA = yaw-startYaw;
    
    
    speedl = Kpl*errorL + Kdl*(errorL - errorLPrev) / (millis() - t);
    speeda = Kpa*errorA + Kda*getRate() ;
    direction = ( speedl >=0 ? true:false );

    // Serial.print(errorL);
    // Serial.print(" ");
    // Serial.print(speedl);
    // Serial.print(" ");
    // Serial.println(speeda);

    analogWrite(leftMotorForward , direction * fixSpeed(speedl - speeda)); // might change the +- signs here
    analogWrite(leftMotorBackward , !direction * fixSpeed(speedl - speeda));              // also might add a constrain fa lw el speed less/greater than the limits, it doesn't overflow

    analogWrite(rightMotorForward , direction * fixSpeed(speedl + speeda));
    analogWrite(rightMotorBackward , !direction * fixSpeed(speedl + speeda)); 

    errorLPrev = errorL;
    errorAPrev = errorA; 
    t = millis();

  }
  //Serial.println(calculateDistance(startX,startY));
  analogWrite(leftMotorForward ,   0);
  analogWrite(leftMotorBackward ,  0);
  analogWrite(rightMotorForward ,  0);
  analogWrite(rightMotorBackward , 0);


}

double fixSpeed(double speed)
{
  speed = constrain(speed, -150, 150);
  if(abs(speed) < 5)return 0;
  if(speed > 0) return map(speed,0,150,45,150);
  if(speed < 0) return map(speed,-150,0,-150,-45);
}

inline double calculateDistance(double x, double y)
{
  return sqrt(pow(x - xPosition,2) + pow(y-yPosition,2));
}

double angleDiff(double start,double goal)
{
  //goal  = (goal + 360) % 360.0;
  goal += (goal>360? -360:0);
  goal += (goal<0? 360:0);
  double angle = goal - start;
  double angle2 = angle + (angle<0? 360:-360);
  return (abs(angle) < abs(angle2)? angle:angle2);
}


inline float getOrientationX()
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return orientationData.orientation.x;
}

inline float getRate()
{
  sensors_event_t gyroData;
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return gyroData.gyro.x;
}


inline void getPosition()
{
  double leftRevolutions = 1.0* leftEncoder.position() / ticksperlafa;
  double rightRevolutions = 1.0* rightEncoder.position() / ticksperlafa;

  double leftDistance = (leftRevolutions - previousLeft) * circumference;
  double rightDistance = (rightRevolutions - previousRight) * circumference;
  double distance = (leftDistance + rightDistance) /2;

  double encoderDelta = (rightDistance - leftDistance) / distance_between_wheels; // won't use that , and this is in radian
  double bnoDelta = (getOrientationX() - yaw);
  double deltaAngle = bnoDelta; // add the encoder delta to it if you want

  yPosition += distance * cos((yaw + deltaAngle/2) *PI/180);
  xPosition += distance * sin((yaw + deltaAngle/2) *PI/180);
  yaw += deltaAngle; // or yaw = getOrientationX();

  //yaw = (yaw+360)%360;

  previousLeft  = leftRevolutions;
  previousRight = rightRevolutions;

}
inline void READIRS()
{
 // implement reading irs
  int i = 0;
    for (const auto &[trig_pin, echo_pin] : ir_array) {
        u_long a = micros();
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(5);
        int val = analogRead(echo_pin);
        digitalWrite(trig_pin, LOW);
        u_long b = micros();
        //delay(100);
        Serial.print(String(val)+" ");
        readings[i] = val;
        i++;
    }
    Serial.println(i);

}


inline void stop()
{
  analogWrite(leftMotorForward ,   0);
  analogWrite(leftMotorBackward ,  0);
  analogWrite(rightMotorForward ,  0);
  analogWrite(rightMotorBackward , 0);
}

