#include <RotaryEncoderPCNT.h>




#define m1p1 27
#define m1p2 14

bool direction = 0;

int brit = 0;


RotaryEncoderPCNT encoder(25, 12);

int old_position;
int position;
//unsigned long tt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(m1p1,OUTPUT);
  pinMode(m1p2,OUTPUT);

  old_position = encoder.position();
  Serial.println(old_position);
  while(!Serial.available()){};
  Serial.println("Start!");
}


void wait(int time)
{
  unsigned long t = millis();
   long tt = t;
  while(millis() - t < time)
  {
    // old_position = encoder.position();
    // Serial.println(old_position);
    position = encoder.position();
    if(abs(position - old_position) >=10)
    {
      long speed = position - old_position;
      long tim = millis() - tt;
      if(tim == 0)tim ++;
      try{
        Serial.println(speed / tim * 1000);
      }
      catch(int x)
      {

      }
      old_position = position;
      tt = millis();
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(12,HIGH);
  // delay(1000);
  // digitalWrite(12,LOW);
  // delay(1000);
//   while(!Serial.available())
//   {
//     old_position = encoder.position();
//     Serial.println(old_position);
//   }
//   Serial.read();
//   while(brit < 255){
//   analogWrite(m1p1,brit*!direction);
//   analogWrite(m1p2,brit*direction);
//   brit += 50;
//   wait(500);
// }
// wait(2000);
// old_position = encoder.position();
// Serial.println(old_position);
// brit = 0;

// direction = !direction;
Serial.println("slow");
analogWrite(m1p1,100*!direction);
analogWrite(m1p2,100*direction);
wait(3000);
Serial.println("fast");
analogWrite(m1p1,250*!direction);
analogWrite(m1p2,250*direction);
wait(3000);
Serial.println("stop");
analogWrite(m1p1,0);
analogWrite(m1p2,0);
wait(3000);
direction = !direction;
}
