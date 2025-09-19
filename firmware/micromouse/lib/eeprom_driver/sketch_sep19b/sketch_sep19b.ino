#include <EEPROM.h>

int modeByte=2; //byte where the mode will be stored
int changePin=11; //button that will trigger menu
int selectorPin=12; //button that will be used to select mode floodfill,right,left
bool menu=false;
char option;
unsigned long interTimer;
// char modes[]={'f','r','l'};
int IN1=17;
int IN2=18;
int IN3=58;
int IN4=56;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(32); // Allocate 512 bytes for EEPROM emulation
  pinMode(selectorPin,INPUT_PULLUP);
  pinMode(changePin,INPUT_PULLUP);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(changePin), toggleMenu, RISING);
  attachInterrupt(digitalPinToInterrupt(selectorPin), modeChooser, RISING); 
  option=EEPROM.read(modeByte);
  Serial.println((char)EEPROM.read(modeByte));

  interTimer = millis();

  // EEPROM.write(modeByte, '0');
  //   EEPROM.commit();
}

void toggleMenu(){
  menu=true;
  Serial.println("Menu");
}

void modeChooser(){
  if(menu && millis() - interTimer > 250){
    option='0'+(option-'0'+1)%3;
    EEPROM.write(modeByte, option);
    EEPROM.commit();
    interTimer = millis();
  }
}

void moveForward(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void moveLeft(){

}

void moveRight(){

}

void loop() {


  // Serial.println((char)EEPROM.read(modeByte));
  // delay(10);
  if(option='f'){

  }
  else if(option='r'){

  }
  else if(option='l'){

  }
  
}
