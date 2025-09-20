#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RotaryEncoderPCNT.h>
#include <EEPROM.h>
// #include <WiFi.h>
// #include <WebServer.h>

// String dataTosend ;

// // Replace with your network credentials
// const char* ssid = "Borham2";
// const char* password = "18046768";



// // Create a web server on port 80
// WebServer server(80);

int modeByte = 2;      //byte where the mode will be stored
int changePin = 11;    //button that will trigger menu
int selectorPin = 12;  //button that will be used to select mode floodfill,right,left
bool menu = false;
char option;
unsigned long interTimer;

inline void getPosition();  // odom
inline float getOrientationX();
inline float getRate();
bool moveF(double tiles);
void turn(double angle);
bool wallLeft();
bool wallRight();
bool wallFront();


#define MAX_H 18
#define MAX_W 18
#define QUEUE_MAX (MAX_H * MAX_W)

// struct queue;
using queue = struct queue;


struct queue {
  short head, tail, size, counter;
  char items[MAX_H * MAX_W];
};
// void log(const String& text) {
//   //Serial.println(text);
// }


char curr_dir = 0;  // 0--> North, 1 --> East, 2 --> South, 3 --> West
char curr_r = 16, curr_c = 1;

int current_run;
int previous_run;

bool maze[MAX_H][MAX_W][5] = { 0 };  // represents the maze, first 4 bits represent the walls N E S W, the last bit represents the visiting status
//leh mn3melsh byte/char maze[MAX_H][MAX_W] ?

short dis[MAX_H][MAX_W] = {
  { 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16 },
  { 15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
  { 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13 },
  { 12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
  { 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
  { 10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
  { 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8 },
  { 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8 },
  { 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
  { 10, 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10 },
  { 11, 10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
  { 12, 11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
  { 13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12, 13 },
  { 14, 13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
  { 15, 14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16 }
};

queue r_q;
queue c_q;








#define ticksperlafa 1400
#define circumference 10.681
#define distance_between_wheels 9
#define PI 3.141592653589



// right motor pins
#define rightMotorForward 15
#define rightMotorBackward 17
RotaryEncoderPCNT rightEncoder(7, 8);
double previousRight;

// left motor pins 17 15
#define leftMotorForward 14
#define leftMotorBackward 13
RotaryEncoderPCNT leftEncoder(10, 9);  // 8 7
double previousLeft;


int left_revolutions, prev_left_revolutions;
int right_revolutions, prev_right_revolutions;


double xPosition = 0, yPosition = 0;
double yaw = 0;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);




constexpr uint8_t ADC1_0 = 1;
constexpr uint8_t ADC1_1 = 2;
constexpr uint8_t ADC1_2 = 3;
constexpr uint8_t ADC1_3 = 4;
constexpr uint8_t ADC1_4 = 5;
constexpr uint8_t ADC1_5 = 6;

#define frontrightThresh 100
#define frontleftThresh 100
#define leftThresh 100
#define rightThresh 100



struct IR {
  uint8_t trig_pin;
  uint8_t echo_pin;
};

IR front_left_ir = { 36, ADC1_0 };
IR front_right_ir = { 33, ADC1_5 };
IR left_ir = { 35, ADC1_2 };
IR right_ir = { 18, ADC1_3 };
IR left_diagonal_ir = { 40, ADC1_1 };
IR right_diagonal_ir = { 34, ADC1_4 };

std::array ir_array = { front_left_ir, front_right_ir, left_ir, right_ir, left_diagonal_ir, right_diagonal_ir };
int readings[6];




String tostr(short x) {
  //log("tostr");
  if (x == 0) return "0";
  String ret = "";
  while (x) {
    ret = char((x % 10) + '0') + ret;
    x /= 10;
  }
  return ret;
}

// queue implementation

void initialise(queue &q, short size) {
  q.head = 0;
  q.tail = 0;
  q.size = size;
  q.counter = 0;
}
bool isfull(queue &q) {
  if (q.counter == q.size)
    return (1);
  else
    return (0);
}
bool isempty(queue &q) {
  if (q.counter == 0)
    return (1);
  else
    return (0);
}

void enqueue(queue &q, char value) {
  if (!isfull(q)) {
    q.items[q.tail] = value;
    q.tail = (q.tail + 1) % q.size;
    q.counter++;
  }
}
char dequeue(queue &q) {
  if (!isempty(q)) {
    char result;
    result = q.items[q.head];
    q.head = (q.head + 1) % q.size;
    q.counter--;
    return result;
  }
}




//end of queue implementation

void update_mms_maze() {
  for (int i = 1; i <= 16; i++) {
    for (int j = 1; j <= 16; j++) {
      //setText(j-1,16-i,tostr(dis[i][j]));
    }
  }
}

void set_wall() {
  // //Serial.println("\ncoloring walls" + String(curr_c - 1) + " " + String(16 - curr_r));
  // for(int i=0;i<4;i++) //Serial.println(maze[curr_r][curr_c][i]);
  //if(maze[curr_r][curr_c][0])setWall(curr_c-1,16-curr_r,'n');
  //if(maze[curr_r][curr_c][1])setWall(curr_c-1,16-curr_r,'e');
  //if(maze[curr_r][curr_c][2])setWall(curr_c-1,16-curr_r,'s');
  //if(maze[curr_r][curr_c][3])setWall(curr_c-1,16-curr_r,'w');
}

// change r, c to move to: N, E, S, W
signed char r_mov[4] = { -1, 0, 1, 0 };
signed char c_mov[4] = { 0, 1, 0, -1 };

bool isValid(char r, char c) {
  return ((r >= 0) && (r < MAX_H)) && ((c >= 0) && (c < MAX_W));
}

bool isAccessible(char r, char c, char dir) {
  return !(maze[r][c][dir] || maze[r + r_mov[dir]][c + c_mov[dir]][(dir + 2) % 4]);
}


void flood(bool goal = 1) {  // make goal = 0 to change the goal to the start

  for (char i = 0; i < MAX_W; i++) {  //initialize all cells with -1
    for (char j = 0; j < MAX_H; j++) {
      dis[i][j] = -1;
    }
  }

  if (goal) {
    for (char x = 4; x < 6; x++) {  //change middle cells with 0
      for (char w = 12; w < 14; w++) {
        dis[x][w] = 0;
        enqueue(r_q, x);
        enqueue(c_q, w);
      }
    }
  } else {
    dis[16][1] = 0;
    enqueue(r_q, 16);
    enqueue(c_q, 1);
  }

  while (!isempty(c_q) && !isempty(r_q)) {
    char r = dequeue(r_q);
    char c = dequeue(c_q);
    for (int i = 0; i < 4; i++) {
      if (isValid(r + r_mov[i], c + c_mov[i]) && isAccessible(r, c, i) && dis[r + r_mov[i]][c + c_mov[i]] == -1) {
        dis[r + r_mov[i]][c + c_mov[i]] = dis[r][c] + 1;
        enqueue(r_q, r + r_mov[i]);
        enqueue(c_q, c + c_mov[i]);
      }
    }
  }
  update_mms_maze();
}


void moveTo(char r, char c) {
  // get where I want to move relative to abolute direction (y3ny lw el robot bases north) ana lesa m2alef el term dah
  short dir;
  if (r < curr_r) dir = 0;
  if (r > curr_r) dir = 2;
  if (c < curr_c) dir = 3;
  if (c > curr_c) dir = 1;
  // //Serial.println(String((int) r)+" "+ String((int) c));
  //log(String("direction: ") + tostr(dir));

  // compare the movement direction to the current directoin to know how should I turn

  if (dir - curr_dir == -1 || dir - curr_dir == 3)  // turn left
  {
    turn(-90);  //turnLeft();
    delay(500);
      //moveForward();
    curr_dir += 3;  // b3mel +3 msh -1 because el negative numbers don't work/work differently fel mod//
    curr_dir %= 4;
    if (!moveF(1)) return; 
  }

  else if (dir - curr_dir == 1 || dir - curr_dir == -3)  // turn right
  {
    turn(90);  //turnRight();
    delay(500);
    curr_dir++;
    curr_dir %= 4;
    if (!moveF(1)) return;  //moveForward(); //this function returns 0 if it was unable to move,so we leave the func, 3shan man8ayarsh el curr c wel curr r

  } else if (dir == curr_dir)  // move forward
  {
    if (!moveF(1)) return;  //moveForward();
  } else                    // turn 180
  {
    //turnRight();
    turn(180);  //turnRight();
    delay(500);
    curr_dir += 2;
    curr_dir %= 4;
    if (!moveF(1)) return;  //moveForward();
  }

  curr_dir %= 4;
  curr_c = c;
  curr_r = r;
  delay(500);
}

void exploreToCenter() {
  while (!(((curr_r == 12) || (curr_r == 13)) && ((curr_c == 4) || (curr_c == 5))) && !menu ) {
    maze[curr_r][curr_c][4] = 1;
    //log("start: " + tostr(dis[curr_r][curr_c]));
    //log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
    bool walls[4];
    walls[0] = wallFront();
    walls[1] = wallRight();
    //walls [2] = wallBack(); wall back isn't working
    walls[3] = wallLeft();
    if (curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
    //walls[2] = 0;
    //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
    //std::cerr<<std::endl;
    // dataTosend = String(wallLeft()) + " " + String(wallFront()) + " " + String(wallRight()) + "\n" ;

    Serial.print((int)curr_c);
    Serial.print(" ");
    Serial.println((int)curr_r);
    Serial.print(wallLeft());
    Serial.print(" ");
    Serial.print(wallFront());
    Serial.print(" ");
    Serial.println(wallRight());

    char d = curr_dir, w = 0;
    do {
      maze[curr_r][curr_c][d] = walls[w];
      d = (d + 1) % 4;
      w++;
    } while (d != curr_dir);

    set_wall();
    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++) {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c]) {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }
    //log("to: " + tostr(dis[next_r][next_c]));

    if ((next_r == curr_r) && (next_c == curr_c))
      flood();
    else moveTo(next_r, next_c);
    //   while (!Serial.available())
    //     ;
    // Serial.read();
    
  }
}

void exploreToStart() {
  while (!(curr_c == 1 && curr_r == 16) && !menu) {
    maze[curr_r][curr_c][4] = 1;
    //log("start: " + tostr(dis[curr_r][curr_c]));
    //log(String((int)curr_r)+" "+String((int)curr_c )+" "+String((int)curr_dir));
    bool walls[4];
    walls[0] = wallFront();
    walls[1] = wallRight();
    //walls [2] = wallBack(); wall back isn't working
    walls[3] = wallLeft();
    if (curr_r == 16 && curr_c == 1 && curr_dir == 0) walls[2] = 1;
    else walls[2] = 0;
    //for(int i=0;i<4;i++) std::cerr<<walls[i]<<" ";
    //std::cerr<<std::endl;



    char d = curr_dir, w = 0;
    do {
      maze[curr_r][curr_c][d] = walls[w];
      d = (d + 1) % 4;
      w++;
    } while (d != curr_dir);

    set_wall();
    char next_r = curr_r, next_c = curr_c;

    for (char i = 0; i < 4; i++) {
      if (isValid(curr_r + r_mov[i], curr_c + c_mov[i]) && isAccessible(curr_r, curr_c, i) && dis[curr_r + r_mov[i]][curr_c + c_mov[i]] < dis[next_r][next_c]) {
        next_r = curr_r + r_mov[i];
        next_c = curr_c + c_mov[i];
      }
    }
    // log("to: " + tostr(dis[next_r][next_c]));

    if ((next_r == curr_r) && (next_c == curr_c))
      flood(0);
    else
      moveTo(next_r, next_c);
  }
}









// void setup() {
//   //Serial.begin(19200);
//   initialise(c_q,MAX_H * MAX_W); //queue initialisation for storing row and coloumn
//   initialise(r_q,MAX_H* MAX_W);
//   update_mms_maze();
//   log("Khalast setup");
// //   //Serial.println("Khalast setup");


// }

// void loop() {
//     log("Dakhalt el loop");
//     while(1) {
//     flood();
//     previous_run = current_run;
//     exploreToCenter ();
//     current_run = dis[16][1];
//     if(current_run != 0 && current_run == previous_run) break;
//     flood(0);
//     exploreToStart ();
//     log("done!!!! The best run is "+ current_run);
//     }
// }


int theoreticalHeading = 0;


inline void READIRS() {
  // implement reading irs

  int i = 0;
  for (const auto &[trig_pin, echo_pin] : ir_array) {
    int lit_val = 0, dark_val = 0;
    for (int j = 0; j < 5; j++) {
      digitalWrite(trig_pin, LOW);
      dark_val += analogRead(echo_pin);
      // u_long a = micros();
      digitalWrite(trig_pin, HIGH);
      delayMicroseconds(5);
      lit_val += analogRead(echo_pin);
      digitalWrite(trig_pin, LOW);
      // u_long b = micros();
      //delay(100);
    }
    Serial.print(String((lit_val - dark_val) / 5) + " ");
    readings[i] = (lit_val - dark_val) / 5;
    i++;
  }
  Serial.println();
}


bool wallFront() {
  for (int i = 0; i < 10; i++) {
    READIRS();
    if (readings[0] > frontleftThresh && readings[1] > frontrightThresh) return 1;
  }
  return 0;
}
bool wallRight() {
  for (int i = 0; i < 10; i++) {
    READIRS();
    if (readings[3] > rightThresh) return 1;
  }
  return 0;
}
bool wallLeft() {
  for (int i = 0; i < 10; i++) {
    READIRS();
    if (readings[2] > leftThresh) return 1;
  }
  return 0;
}




//1 4
void turn(double angle) {
  getPosition();
  double desiredAngle = theoreticalHeading + angle;
  double error = angleDiff(yaw, desiredAngle);
  bool direction = (error > 0 ? true : false);  // true -> turn right | false -> turn left
  double errorPrev = error;
  double totalerror = 0;
  unsigned long t = millis();

  double kp = 1.5;  // Kp and Kd will be set with testing
  double kd = -7;

  double speed = 100;

  int counter = 0;

  while (abs(error) > 2 || fabs(getRate()) > 0.5) {
    getPosition();
    error = angleDiff(yaw, desiredAngle);

    Serial.print(desiredAngle);
    Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(error);
    speed = kp * error + kd * getRate();
    speed = fixSpeed(speed);
    // //Serial.print(" ");
    // //Serial.print(speed);
    // //Serial.print(" ");
    // //Serial.println(getRate());
    direction = (speed > 0 ? true : false);
    analogWrite(leftMotorForward, (direction)*abs(speed));
    analogWrite(leftMotorBackward, (!direction) * abs(speed));

    analogWrite(rightMotorForward, (!direction) * abs(speed));
    analogWrite(rightMotorBackward, (direction)*abs(speed));


    errorPrev = error;
    totalerror += error;
    t = millis();
    if (abs(getRate()) < 0.1) counter++;
    if (counter >= 40) break;
  }
  getPosition();
  //Serial.println(yaw);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  theoreticalHeading += angle;
  theoreticalHeading = (theoreticalHeading + 360) % 360;
}
//3 -1
//04 -1


//2 -1
//-1.2 1.5
bool moveF(double tiles = 16)           // if you want to move tile by tile use moveF(1), if you want continuous use moveF();
{                                       // just need to add to make it stop using the irs
  double desiredDistance = tiles * 19;  // el tile el mafrood 18cm, bas we found it would move slightly less than what we wanted, fa we increased it

  double startX = xPosition, startY = yPosition;
  double startYaw = theoreticalHeading;

  unsigned long startRight = rightEncoder.position();
  unsigned long startLeft = leftEncoder.position();
  unsigned long rightTicks, leftTicks;
  unsigned long startTime = millis();

  // for the distance
  double errorL = desiredDistance - calculateDistance(startX, startY);
  double errorLPrev = errorL;

  bool direction = (errorL >= 0 ? true : false);  // true -> forward, false -> backward

  // to keep moving staight
  double errorA = angleDiff(yaw, startYaw);
  double errorAPrev = errorA;


  //double errorTicks = 0;
  double errorTicksPrev = 0;

  unsigned long t = millis();

  double Kpl = 3;  // KD AND KP are changed with testing
  double Kdl = -1;

  double Kpa = -2.95;  // changed
  double Kda = 1.2;    // decreased

  double KpTicks = 0.0;
  double KdTicks = 0.0;
  double kiTicks = 0.0;


  double speedl;
  double speeda;
  double speedTicks = 0;
  double speed;
  //Serial.println(errorL);

  //unsigned long  timeout_timer = millis();
  char timeout_ctr = 0;

  while ((abs(errorL) > 0.2) && timeout_ctr < 50)  // this 1 might change
  {
    //READIRS();
    rightTicks = rightEncoder.position() - startRight;
    leftTicks = leftEncoder.position() - startLeft;
    unsigned long deltaTicks = rightTicks - leftTicks;
    // static double integralval = 0 ;
    // if (fabs(integralval) > 255)
    //     integralval += kiTicks*(deltaTicks - errorTicksPrev)*(millis() - t) ;
    // speedTicks = KpTicks*deltaTicks + KdTicks * (deltaTicks - errorTicksPrev)/(millis() - t)  + integralval;

    getPosition();
    errorL = desiredDistance - calculateDistance(startX, startY);
    errorA = angleDiff(yaw, startYaw);


    speedl = Kpl * errorL + Kdl * (errorL - errorLPrev) / (millis() - t);
    speeda = Kpa * errorA + Kda * getRate();



    direction = (speedl >= 0 ? true : false);

    //Serial.print(errorL);
    //Serial.print(" ");
    Serial.print(yaw);
    Serial.print(" ");
    //Serial.print(speedl);
    Serial.print(fixSpeed(speedl - speeda));
    Serial.print(" ");
    //Serial.println(speeda);
    Serial.print(fixSpeed(speedl + speeda));

    Serial.print(" ");

    analogWrite(leftMotorForward, direction * abs(fixSpeed(speedl - speeda)));      // might change the +- signs here
    analogWrite(leftMotorBackward, (!direction) * abs(fixSpeed(speedl - speeda)));  // also might add a constrain fa lw el speed less/greater than the limits, it doesn't overflow

    analogWrite(rightMotorForward, direction * abs(fixSpeed(speedl + speeda)));
    analogWrite(rightMotorBackward, (!direction) * abs(fixSpeed(speedl + speeda)));

    errorLPrev = errorL;
    errorAPrev = errorA;
    errorTicksPrev = deltaTicks;

    t = millis();
    //if(t - startTime > 5000)
    if (fabs(getLin()) < 0.1) {
      // timeout_timer = millis();
      timeout_ctr++;
    }
    Serial.println(getLin());
  }

  Serial.print("Done ");
  //   Serial.println(calculateDistance(startX,startY));
  Serial.println(errorL);

  analogWrite(rightMotorForward, 0);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorBackward, 0);

  analogWrite(rightMotorForward, 255);
  analogWrite(leftMotorForward, 255);
  analogWrite(rightMotorBackward, 255);
  analogWrite(leftMotorBackward, 255);
  // delayMicroseconds(20);
  delay(50);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  if (timeout_ctr >= 50) return 0;
  return 1;
}

double fixSpeed(double speed) {
  int maxx = 100;
  speed = constrain(speed, -maxx, maxx);
  if (abs(speed) < 5) return 0;
  if (speed > 0) return map(speed, 0, maxx, 45, maxx);
  if (speed < 0) return map(speed, -maxx, 0, -maxx, -45);
}

inline double calculateDistance(double x, double y) {
  return sqrt(pow(x - xPosition, 2) + pow(y - yPosition, 2));
}

double angleDiff(double start, double goal) {
  //goal  = (goal + 360) % 360.0;
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
  return gyroData.gyro.x;  // might change it to gyro.z msh x , haven't tested yet -----------------------------------------------------------------------IMPORTANT
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

  double encoderDelta = (rightDistance - leftDistance) / distance_between_wheels;  // won't use that , and this is in radian
  double bnoDelta = (getOrientationX() - yaw);
  double deltaAngle = bnoDelta;  // add the encoder delta to it if you want

  yPosition += distance * cos((yaw + deltaAngle / 2) * PI / 180);
  xPosition += distance * sin((yaw + deltaAngle / 2) * PI / 180);
  yaw += deltaAngle;  // or yaw = getOrientationX();

  //yaw = (yaw+360)%360;

  previousLeft = leftRevolutions;
  previousRight = rightRevolutions;
}










// String generateHTML() {

//   String html = "<!DOCTYPE html><html>";
//   html += "<head><meta http-equiv='refresh' content='2'/>";
//   html += "<title>ESP32 Sensor Debug</title></head>";
//   html += "<body><h1>ESP32 Sensor Data</h1>";
//   html += "<p>Sensor Value: " + dataTosend + "</p>";
//   html += "</body></html>";

//   return html;
// }

// void handleRoot() {
//   server.send(200, "text/html", generateHTML());
// }


void toggleMenu() {
  menu = true;
  Serial.println("Menu");
}

void modeChooser() {
  if (menu && millis() - interTimer > 250) {
    Serial.print("changing mode");
    option = '0' + (option - '0' + 1) % 3;
    EEPROM.write(modeByte, option);
    EEPROM.commit();
    interTimer = millis();
  } 
}


void setup() {
  //   // put your setup code here, to run once:
  Serial.begin(115200);

  EEPROM.begin(32);  // Allocate 512 bytes for EEPROM emulation
  pinMode(selectorPin, INPUT_PULLUP);
  pinMode(changePin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(changePin), toggleMenu, RISING);
  attachInterrupt(digitalPinToInterrupt(selectorPin), modeChooser, RISING);
  option = EEPROM.read(modeByte);
  if (option < '0' || option > '2') option = '1';


  //   delay(1000);

  //   WiFi.begin(ssid, password);
  //   Serial.print("Connecting to WiFi");
  //   while (WiFi.status() != WL_CONNECTED) {
  //     delay(500);
  //     Serial.print(".");
  //   }

  //   Serial.println("\nWiFi connected.");
  //   Serial.print("IP address: ");
  //   Serial.println(WiFi.localIP());

  //   server.on("/", handleRoot);
  //   server.begin();
  //   Serial.println("HTTP server started");

  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  // analogWriteResolution(leftMotorForward, 10);
  // analogWriteResolution(leftMotorBackward, 10);
  // analogWriteResolution(rightMotorForward, 10);
  // analogWriteResolution(rightMotorBackward, 10);



  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);

  //delay(5000);

  // Serial.begin(115200);
  initialise(c_q, MAX_H * MAX_W);  //queue initialisation for storing row and coloumn
  initialise(r_q, MAX_H * MAX_W);
  update_mms_maze();
  //log("Khalast setup");
  Wire.begin(21, 16);
  bno = Adafruit_BNO055(55, 0x29, &Wire);
  //while (!Serial) delay(10);
  //Serial.println("start");
  if (!bno.begin())  // lol
  {
    //Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  //delay(1000);
  bno.setExtCrystalUse(true);
  //Serial.println("done withe the bno");
  // digitalWRite(leftMotorForward , HIGH);
  // digitalWrite(rightMotorForward,HIGH);

  leftEncoder.setPosition(0);
  rightEncoder.setPosition(0);
  getPosition();

  for (const auto &[trig_pin, echo_pin] : ir_array) {
    pinMode(trig_pin, OUTPUT);
    digitalWrite(trig_pin, LOW);
    pinMode(echo_pin, INPUT);
  }
  delay(2000);

  interTimer = millis();
  //Serial.println("Done with the irs");


  Serial.println("Done with the setup");
}




void loop() {
  // put your main code here, to run repeatedly:
  // moveF(1);
  //turn(90);
  //   getPosition();
  //   analogWrite(leftMotorForward, 0);
  //   analogWrite(leftMotorBackward, 0);
  //   analogWrite(rightMotorForward, 0);
  //   analogWrite(rightMotorBackward, 0);
  // delay(2000);
  // analogWrite(leftMotorForward, 400);
  // analogWrite(leftMotorBackward, 0);
  // analogWrite(rightMotorForward, 400);
  // analogWrite(rightMotorBackward, 0);
  // delay(2000);
  // Serial.print(leftEncoder.position());
  // Serial.print(" ");
  // Serial.println(rightEncoder.position());
  //Serial.println(getRate());


  //   READIRS();
  // if(readings[3] < 100)
  // {
  //     turn(90);
  //     // theoreticalHeading = (theoreticalHeading + 90) % 360;
  //     delay(500);
  //     moveF(1);
  //     Serial.println("RIGHT");
  // }
  // else if (readings[0] < 100)
  // {
  //     moveF(1);
  //     Serial.println("FORWARD");
  // }
  // else
  // {
  //     turn(-90);
  //     Serial.println("LEFT");
  //     // theoreticalHeading = (theoreticalHeading + 270) % 360;
  // }
  // delay(1000);
  // moveF(1);
  // delay(1000);
  // getPosition();
  // Serial.print(xPosition);
  // Serial.print(" ");
  // Serial.print(yPosition);
  // Serial.print(" ");
  // Serial.println(yaw);
  // delay(100);
  // READIRS();

  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 90;
  // delay(500);
  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 180;
  // delay(500);
  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 270;
  // delay(500);
  // moveF(1);
  // delay(500);
  // turn(90);
  // theoreticalHeading = 0;
  // delay(500);

  // moveF(2);
  // delay(100);
  // turn(180);
  // delay(100);
  // moveF(2);
  // delay(100);
  // turn(-180);
  // delay(100);

  // Serial.print(wallLeft());
  // Serial.print(" ");
  // Serial.print(wallFront());
  // Serial.print(" ");
  // Serial.println(wallRight());

  //   while(1) {
  //   // server.handleClient();
  //   flood();
  //   previous_run = current_run;
  //   exploreToCenter ();
  //   current_run = dis[16][1];
  //   if(current_run != 0 && current_run == previous_run) break;
  //   flood(0);
  //   exploreToStart ();
  //   //log("done!!!! The best run is "+ current_run);
  //   }


///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // option --> correspondance
  // 0 --> floodfill
  // 1 --> right-hand
  // 2 --> left-hand
  if (menu) {
    if (option == '0') {
      Serial.println("0 menu");
      // delay(2000);
      analogWrite(leftMotorForward, 100);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 100);
      analogWrite(rightMotorBackward, 0);
    } else if (option == '1') {
      analogWrite(leftMotorForward, 100);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 0);
      analogWrite(rightMotorBackward, 100);
    } else if (option == '2') {
      analogWrite(leftMotorForward, 0);
      analogWrite(leftMotorBackward, 100);
      analogWrite(rightMotorForward, 100);
      analogWrite(rightMotorBackward, 0);
    } else {
      analogWrite(leftMotorForward, 0);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 0);
      analogWrite(rightMotorBackward, 0);
      Serial.println("WTF HAPPENED?? MENU " + String(option));
    }
  } else {
    if (option == '0') {
      Serial.println("0 no menu");
      delay(2000);
      while (!menu) {
        // server.handleClient();
        Serial.println("0 while");
      // delay(2000);
        flood();
        Serial.println("done flood");
        previous_run = current_run;
        exploreToCenter();
        Serial.println("done exploretocenter");
        current_run = dis[16][1];
        if (current_run != 0 && current_run == previous_run) break;
        flood(0);
        exploreToStart();
        //log("done!!!! The best run is "+ current_run);
      }
    } else if (option == '1') {
      getPosition();
      analogWrite(leftMotorForward, 0);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 0);
      analogWrite(rightMotorBackward, 0);
      delay(1000);
      READIRS();
      if(!wallRight()){//if (readings[3] < 100) {
        turn(90);
        // theoreticalHeading = (theoreticalHeading + 90) % 360;
        delay(500);
        moveF(1);
        Serial.println("RHRIGHT");
      } else if (!wallFront()){//else if (readings[0] < 100) {
        moveF(1);
        Serial.println("RHFORWARD");
      } else {
        turn(-90);
        Serial.println("RHLEFT");
        // theoreticalHeading = (theoreticalHeading + 270) % 360;
      }
    } else if (option == '2') {
      getPosition();
      analogWrite(leftMotorForward, 0);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 0);
      analogWrite(rightMotorBackward, 0);
      delay(1000);
      READIRS();
      if(!wallLeft()){//if (readings[2] < 100) {
        turn(-90);
        // theoreticalHeading = (theoreticalHeading + 90) % 360;
        delay(500);
        moveF(1);
        Serial.println("LHLEFT");
      } else if (!wallFront()){//else if (readings[0] < 100) {
        moveF(1);
        Serial.println("LHFORWARD");
      } else {
        turn(90);
        Serial.println("RHLEFT");
        // theoreticalHeading = (theoreticalHeading + 270) % 360;
      }
    } else {
      analogWrite(leftMotorForward, 0);
      analogWrite(leftMotorBackward, 0);
      analogWrite(rightMotorForward, 0);
      analogWrite(rightMotorBackward, 0);
      Serial.println("WTF HAPPENED?? NO MENU " + String(option));
    }
  }
  //READIRS();
}