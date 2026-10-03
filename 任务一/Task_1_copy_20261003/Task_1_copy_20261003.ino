/*
  任务一 1.1和1.2：摇杆控制机械臂

  功能：
  1. A0、A1、A2、A3 四路摇杆分别控制底座、前臂、后臂和夹爪。
  2. 上位机发送 O：夹爪张开，发送 S：夹爪关闭。
  3. 上位机发送 H：提高整体运行速度，发送 L：降低整体运行速度。

  速度说明：
  DSD 是舵机每改变 1 度后的等待时间
  DSD 越小，机械臂运行越快

  更换机械臂时：
  先修改下面的舵机 PWM 引脚、摇杆输入引脚和舵机角度范围，
  再根据实际机械限位调试 cOpen、cClose 和初始角度。
*/

#include <Servo.h>

// b底座，f前臂，r后臂，c夹爪
Servo base, fArm, rArm, claw;

//舵机 PWM 引脚，换机械臂先改这里
const int bPin = 9;  // b底座
const int fPin = 8;  // f前臂
const int rPin = 6;  // r后臂
const int cPin = 7;  // c夹爪

//摇杆输入引脚，换控制器先改这里
const int bJoy = A0; // b底座摇杆
const int fJoy = A3; // f前臂摇杆
const int rJoy = A1; // r后臂摇杆
const int cJoy = A2; // c夹爪摇杆

//舵机安全角度范围，换机械臂先改这里
const int bMin = 0;   const int bMax = 180; // b底座
const int fMin = 15;  const int fMax = 120; // f前臂
const int rMin = 40;  const int rMax = 170; // r后臂
const int cMin = 5;   const int cMax = 90;  // c夹爪

//夹爪开合角度，实物方向相反时交换 cOpen 和 cClose
const int cOpen = cMin;   // 爪子张开：5度
const int cClose = cMax;  // 爪子关闭：90度

//上电后的初始角度
int bPos = 90;
int fPos = 90;
int rPos = 90;
int cPos = 90; 

//摇杆参数
const int joyCenter = 512; // 摇杆中心值
const int joyDead = 100;   // 摇杆死区，防止松手时抖动
const int joyStep = 1;     // 每次循环改变的角度

//速度参数：DSD 是等待时间，越小越快
int DSD = 15;
const int DSDmin = 2;    // DSD 最小值，速度最快
const int DSDmax = 60;   // DSD 最大值，速度最慢
const int DSDstep = 5;   //每次调整的数值

// 将角度限制在安全范围内
int limitData(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

// 使用 switch case 给指定舵机写入角度
void servoWrite(char name, int value) {
  switch (name) {
    case 'b':
      bPos = limitData(value, bMin, bMax);
      base.write(bPos);
      break;
    case 'f':
      fPos = limitData(value, fMin, fMax);
      fArm.write(fPos);
      break;
    case 'r':
      rPos = limitData(value, rMin, rMax);
      rArm.write(rPos);
      break;
    case 'c':
      cPos = limitData(value, cMin, cMax);
      claw.write(cPos);
      break;
    default:
      break;
  }
}

// 使用 switch case 让指定舵机改变一个角度
void moveJoint(char name, int data) {
  switch (name) {
    case 'b':
      servoWrite('b', bPos + data);
      break;
    case 'f':
      servoWrite('f', fPos + data);
      break;
    case 'r':
      servoWrite('r', rPos + data);
      break;
    case 'c':
      servoWrite('c', cPos + data);
      break;
    default:
      break;
  }
}

// 让指定舵机逐度运行到目标角度，DSD 决定运行速度
void moveTo(char name, int target) {
  int now = 0;
  int minValue = 0;
  int maxValue = 180;

  switch (name) {
    case 'b':
      now = bPos;
      minValue = bMin;
      maxValue = bMax;
      break;
    case 'f':
      now = fPos;
      minValue = fMin;
      maxValue = fMax;
      break;
    case 'r':
      now = rPos;
      minValue = rMin;
      maxValue = rMax;
      break;
    case 'c':
      now = cPos;
      minValue = cMin;
      maxValue = cMax;
      break;
    default:
      return;
  }

  target = limitData(target, minValue, maxValue);

  while (now != target) {
    if (now < target) now++;
    else now--;
    servoWrite(name, now);
    delay(DSD);
  }
}

// 读取摇杆方向：1 正向，-1 反向，0 停止
int joyDir(int pin) {
  int joyData = analogRead(pin) - joyCenter;
  if (joyData > joyDead) return 1;
  if (joyData < -joyDead) return -1;
  return 0;
}

// 摇杆独立控制四个舵机
void joyCtrl() {
  int dir;

  dir = joyDir(bJoy);
  if (dir != 0) moveJoint('b', -1*dir * joyStep);

  dir = joyDir(fJoy);
  if (dir != 0) moveJoint('f', -1*dir * joyStep);

  dir = joyDir(rJoy);
  if (dir != 0) moveJoint('r', -1*dir * joyStep);

  dir = joyDir(cJoy);
  if (dir != 0) moveJoint('c', dir * joyStep);
}

// 处理上位机固定指令
void armDataCmd(char name) {
  // 允许上位机发送小写字母
  if (name >= 'a' && name <= 'z') name = name - 'a' + 'A';

  switch (name) {
    case 'O':
      moveTo('c', cOpen);
      Serial.println("爪子张开");
      break;

    case 'S':
      moveTo('c', cClose);
      Serial.println("爪子关闭");
      break;

    case 'H':
      DSD = DSD - DSDstep;
      if (DSD < DSDmin) DSD = DSDmin;
      Serial.print("提高速度，DSD=");
      Serial.println(DSD);
      break;

    case 'L':
      DSD = DSD + DSDstep;
      if (DSD > DSDmax) DSD = DSDmax;
      Serial.print("降低速度，DSD=");
      Serial.println(DSD);
      break;

    default:
      Serial.println("无法识别指令");
      break;
  }
}

// 读取上位机发送的单字符指令
void readCmd() {
  while (Serial.available() > 0) {
    char name = Serial.read();
    if (name != '\r' && name != '\n') armDataCmd(name);
  }
}

void setup() {
  // 初始化电机对应 PWM 针脚
  base.attach(bPin);
  delay(200);
  fArm.attach(fPin);
  delay(200);
  rArm.attach(rPin);
  delay(200);
  claw.attach(cPin);
  delay(200);

  // 初始化串口
  Serial.begin(9600);
  Serial.println("欢迎使用");

  // 机械臂上电先回到设定的初始角度
  servoWrite('b', bPos);
  servoWrite('f', fPos);
  servoWrite('r', rPos);
  servoWrite('c', cPos);
}

void loop() {
  readCmd(); // 先处理上位机固定指令
  joyCtrl(); // 再读取摇杆
  delay(DSD); // DSD 是机械臂整体运行的等待时间
}
