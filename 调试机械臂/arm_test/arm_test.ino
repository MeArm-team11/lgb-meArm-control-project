#include <Servo.h>
Servo base,fArm,rArm,claw;//b底座，f前臂，r后臂，c夹爪
//存储当前电机角度数值
int bPos=90;
int fPos=90;
int rPos=90;
int cPos=90;
//限制点击转动角度,用const创建常量
const int bMin=0;
const int bMax=180;
const int fMin=15;
const int fMax=120;
const int rMin=40;
const int rMax=170;
const int cMin=5;
const int cMax=90;
//初始化电机对应pwm针脚
void setup() {
  base.attach(9);
  delay(200);
  fArm.attach(8);
  delay(200);
  rArm.attach(6);
  delay(200);
  claw.attach(7);
  delay(200);
  Serial.begin(9600);
  Serial.println("欢迎使用");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    char name = Serial.read();
    armDataCmd(name);
  }
  base.write(bPos);
  delay(15);
  rArm.write(rPos);
  delay(15);
  fArm.write(fPos);
  delay(15);
  claw.write(cPos);
  delay(15);
}
void armDataCmd(char name){
  Serial.print("name=");
  Serial.println(name);
  int servoData = Serial.parseInt();
  switch (name){
    case 'b':
      bPos=servoData;
      Serial.print("转到："); 
      Serial.println(bPos);
      break;
      
      case 'f':
      fPos=servoData;
      Serial.print("转到："); 
      Serial.println(fPos);
      break;

      case 'r':
      rPos=servoData;
      Serial.print("转到："); 
      Serial.println(rPos);
      break;

      case 'c':
      cPos=servoData;
      Serial.print("转到："); 
      Serial.println(cPos);
      break;
  }
}
