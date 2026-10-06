/*
任务二：A/B/C 自动夹取与放置
  任务二目前仍以任务一的控制逻辑为基础

  接线：
  底座舵机 D9，前臂舵机 D8，后臂舵机 D6，夹爪舵机 D7。
  底座摇杆 A0，前臂摇杆 A3，后臂摇杆 A1，夹爪摇杆 A2。

  串口波特率：9600。
  x,y,z 指令以回车或换行结束。
  DSD 是每移动 1 度后的等待时间，DSD 越小，运行越快。

  使用前应核对舵机转向、机械限位和夹爪开合角度。
*/

#include <Servo.h>
#include <stdio.h>

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
const int fMin = 0;  const int fMax = 150; // f前臂15 120
const int rMin = 0;  const int rMax = 180; // r后臂
const int cMin = 5;   const int cMax = 80;  // c夹爪

//夹爪开合角度，实物方向相反时交换 cOpen 和 cClose
const int cOpen = cMin;   // 爪子张开：5度
const int cClose = cMax;  // 爪子关闭：90度

//上电后的初始角度
int bPos = 90;
int fPos = 90;
int rPos = 90;
int cPos = 80;

// 上电初始角度，也是任务三的回中目标
const int bHome = 90;
const int fHome = 90;
const int rHome = 90;
const int cHome = 80;

//摇杆参数
const int joyCenter = 512; // 摇杆中心值
const int joyDead = 100;   // 摇杆死区，防止松手时抖动
const int joyStep = 1;     // 每次循环改变的角度

//速度参数：DSD 是等待时间，越小越快
int DSD = 15;
const int DSDmin = 2;    // DSD 最小值，速度最快
const int DSDmax = 60;   // DSD 最大值，速度最慢
const int DSDstep = 5;   //每次调整的数值
// 自动动作每一步结束后的等待时间，实测后调整
const int actionWait = 300;

// 结构体记录一个位置的三个舵机角度
struct JointPoint {
  int b;
  int f;
  int r;
};

// 记录一个物体的取放位置和夹爪角度
struct ActionData {
  JointPoint getPoint;
  JointPoint putPoint;
  int open;
  int close;
};

// 录制时保存的一帧：四个舵机角度和录制后的时间
struct RecordFrame {
  byte b;
  byte f;
  byte r;
  byte c;
  unsigned long timeMs;
};

// 录制容量和采样间隔
const int recordMax = 80;
const unsigned long recordStep = 200;

// 保存录制的动作帧
RecordFrame recordData[recordMax];

// 录制状态
bool recording = false;
unsigned long recordStart = 0;
unsigned long lastSample = 0;
int frameCount = 0;

// 切换录制状态：第一次开始，第二次停止
void recordCmd() {
  if (!recording) {
    // 开始新一段录制，旧数据不再作为有效记录
    frameCount = 0;
    recordStart = millis();
    lastSample = recordStart;
    recording = true;

    // 先保存开始录制时的姿态
    saveFrame();

    Serial.println("开始录制");
  } else {
    // 保存停止时的姿态，数组已满时 saveFrame() 不再写入
    saveFrame();
    recording = false;

    Serial.print("录制结束，帧数=");
    Serial.println(frameCount);
  }
}

// 任务二的公共安全抬升位置，底座不管
int fHigh = 120;
int rHigh = 85;

// A、B、C 的取物点和放置点
ActionData actionA = {
  {90, 90, 90}, // 取物点：底座、前臂、后臂
  {90, 90, 90}, // 放置点：底座、前臂、后臂
  5,           // 夹爪张开角度
  90           // 夹爪夹紧角度
};

ActionData actionB = {
  {90, 90, 90},
  {90, 90, 90},
  5,
  90
};

ActionData actionC = {
  {90, 90, 90},
  {90, 90, 90},
  5,
  90
};

// 暂存一整行串口指令，例如 x10,y30,z20
char serialLine[40];
int lineLength = 0;

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

// 输出四个舵机当前状态
void printStatus() {
  Serial.print("x");
  Serial.print(bPos);
  Serial.print(",y");
  Serial.print(fPos);
  Serial.print(",z");
  Serial.println(rPos);

  Serial.print("c");
  Serial.println(cPos);
}

// 录制期间，按时间间隔保存动作帧
void recordCtrl() {
  if (!recording) return;

  unsigned long now = millis();

  if (now - lastSample >= recordStep) {
    lastSample = now;
    saveFrame();

    // 存满后自动结束，不再继续写入
    if (frameCount >= recordMax) {
      recording = false;
      Serial.println("录制容量已满，自动停止");
      Serial.print("帧数=");
      Serial.println(frameCount);
    }
  }
}

// 保存当前四个舵机的设定角度和录制时间
void saveFrame() {
  // 数组已满，不继续写入
  if (frameCount >= recordMax) return;

  recordData[frameCount].b = bPos;
  recordData[frameCount].f = fPos;
  recordData[frameCount].r = rPos;
  recordData[frameCount].c = cPos;
  recordData[frameCount].timeMs = millis() - recordStart;

  frameCount++;
}

// 按录制时间播放四个舵机的角度
void playRecord() {
  // 录制期间不允许播放
  if (recording) {
    Serial.println("请先停止录制");
    return;
  }

  // 至少需要两帧
  if (frameCount < 2) {
    Serial.println("没有足够的录制数据");
    return;
  }

  // 当前姿态与第一帧不一致时，先自动回到录制起点
  if (bPos != recordData[0].b || fPos != recordData[0].f ||
      rPos != recordData[0].r || cPos != recordData[0].c) {
    Serial.println("正在返回录制起点");
    // 底座转到第一帧的方向
    moveTo('b', recordData[0].b);
    delay(actionWait);

    // 大小臂移动到第一帧的位置
    moveArm(recordData[0].f, recordData[0].r);
    delay(actionWait);

    // 夹爪恢复第一帧的角度
    moveTo('c', recordData[0].c);
    delay(actionWait);
  }

  Serial.println("开始播放");

  unsigned long playStart = millis();

  for (int i = 0; i < frameCount; i++) {
    // 以第一帧为播放时间起点
    unsigned long frameTime =
        recordData[i].timeMs - recordData[0].timeMs;

    // 等到这一帧对应的时刻
    while (millis() - playStart < frameTime) {
      delay(1);
    }

    // 写入这一帧的四个角度
    servoWrite('b', recordData[i].b);
    servoWrite('f', recordData[i].f);
    servoWrite('r', recordData[i].r);
    servoWrite('c', recordData[i].c);
  }

  Serial.println("播放指令执行结束");
  printStatus();
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

// 让前臂和后臂逐度运行到目标角度，底座和夹爪保持不动
void moveArm(int fTarget, int rTarget) {
  fTarget = limitData(fTarget, fMin, fMax);
  rTarget = limitData(rTarget, rMin, rMax);

  while (fPos != fTarget || rPos != rTarget) {
    if (fPos < fTarget) servoWrite('f', fPos + 1);
    else if (fPos > fTarget) servoWrite('f', fPos - 1);

    if (rPos < rTarget) servoWrite('r', rPos + 1);
    else if (rPos > rTarget) servoWrite('r', rPos - 1);

    delay(DSD);
  }
}

// 根据指定物体的数据，完成夹取和放置
void doAction(ActionData data) {
  // 1. 大小臂抬到安全姿态，底座不动
  moveArm(fHigh, rHigh);
  delay(actionWait);

  // 2. 打开夹爪
  moveTo('c', data.open);
  delay(actionWait);

  // 3. 底座转到取物方向，大小臂不动
  moveTo('b', data.getPoint.b);
  delay(actionWait);

  // 4. 大小臂移动到取物点，底座不动
  moveArm(data.getPoint.f, data.getPoint.r);
  delay(actionWait);

  // 5. 夹紧物体
  moveTo('c', data.close);
  delay(actionWait);

  // 6. 大小臂抬到安全姿态，底座不动
  moveArm(fHigh, rHigh);
  delay(actionWait);

  // 7. 底座转到放置方向，大小臂不动
  moveTo('b', data.putPoint.b);
  delay(actionWait);

  // 8. 大小臂移动到放置点，底座不动
  moveArm(data.putPoint.f, data.putPoint.r);
  delay(actionWait);

  // 9. 打开夹爪，放下物体
  moveTo('c', data.open);
  delay(actionWait);

  // 10. 大小臂再次抬到安全姿态
  moveArm(fHigh, rHigh);
  delay(actionWait);
}

// 下次按键1要执行的物体
char nextObject = 'A';

// 每调用一次，依次选择 A、B、C
void nextAction() {
  switch (nextObject) {
    case 'A':
      Serial.println("按键1选择A");
      // doAction(actionA); // 待A完成标定后启用
      nextObject = 'B';
      break;

    case 'B':
      Serial.println("按键1选择B");
      // doAction(actionB); // 待B完成标定后启用
      nextObject = 'C';
      break;

    case 'C':
      Serial.println("按键1选择C");
      // doAction(actionC); // 待C完成标定后启用
      nextObject = 'A';
      break;
  }
}

// 回到上电初始姿态，先抬大小臂，再转底座
void goHome() {
  // 1. 大小臂抬到安全姿态，底座和夹爪不动
  moveArm(fHigh, rHigh);
  delay(actionWait);

  // 2. 底座回到初始方向
  moveTo('b', bHome);
  delay(actionWait);

  // 3. 大小臂回到初始角度
  moveArm(fHome, rHome);
  delay(actionWait);

  // 4. 夹爪回到初始角度
  moveTo('c', cHome);
  delay(actionWait);
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
  if (dir != 0) moveJoint('b', -dir * joyStep);

  dir = joyDir(fJoy);
  if (dir != 0) moveJoint('f', -dir * joyStep);

  dir = joyDir(rJoy);
  if (dir != 0) moveJoint('r', -dir * joyStep);

  dir = joyDir(cJoy);
  if (dir != 0) moveJoint('c', dir * joyStep);
}

// 处理上位机固定OSHL等指令
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
    
    case 'P':
      printStatus();//输出状态
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

    case 'A':
      Serial.println("开始执行A");
      doAction(actionA);
      Serial.println("A动作指令执行结束");
      break;

    case 'B':
      Serial.println("开始执行B");
      doAction(actionB);
      Serial.println("B动作指令执行结束");
      break;

    case 'C':
      Serial.println("开始执行C");
      doAction(actionC);
      Serial.println("C动作指令执行结束");
      break;
   
    case 'M':
      Serial.println("开始回中");
      goHome();
      Serial.println("回中指令执行结束");
      printStatus();
      break;

    case 'N':
      nextAction();
      break;

    case 'R':
      recordCmd();
      break;

    case 'T':
      playRecord();
      break;

    default:
      Serial.println("无法识别指令");
      break;
  }
}

//把从串口中读取的字符串按照格式赋值给xyz,并且调整舵机角度
void armXYZCmd() {
  int x, y, z;
  char extra;

  int count = sscanf(serialLine, " x%d , y%d , z%d %c",&x, &y, &z, &extra);

  if (count == 3) {
    // 先把目标角度限制在各关节允许的范围内
    x = limitData(x, bMin, bMax);
    y = limitData(y, fMin, fMax);
    z = limitData(z, rMin, rMax);

    // 每轮让三个关节分别向目标移动 1 度
    while (bPos != x || fPos != y || rPos != z) {
      if (bPos < x) servoWrite('b', bPos + 1);
      else if (bPos > x) servoWrite('b', bPos - 1);

      if (fPos < y) servoWrite('f', fPos + 1);
      else if (fPos > y) servoWrite('f', fPos - 1);

      if (rPos < z) servoWrite('r', rPos + 1);
      else if (rPos > z) servoWrite('r', rPos - 1);

      delay(DSD);
    }

    Serial.print("x=");
    Serial.print(bPos);
    Serial.print(" y=");
    Serial.print(fPos);
    Serial.print(" z=");
    Serial.println(rPos);
  } else {
    Serial.println("指令格式错误");
  }
}

// 读取上位机发送的字符串指令
void readCmd() {
  while (Serial.available() > 0) {
    char data = Serial.read();

    if (data == '\r' || data == '\n') {
      if (lineLength > 0 && lineLength < 40) {
        serialLine[lineLength] = '\0';
        armXYZCmd();
      } else if (lineLength == 40) {
        Serial.println("指令太长");
      }
      lineLength = 0;
    } else {
      char name = data;
      if (name >= 'a' && name <= 'z') name = name - 'a' + 'A';

      if (name == 'O' || name == 'S' || name == 'H' || name == 'L' || 
          name == 'P' || name == 'A' || name == 'B' || name == 'C' || 
          name == 'M' || name == 'N' || name == 'R' || name == 'T') {
        armDataCmd(data);
      } else if (lineLength < 39) {
        serialLine[lineLength] = data;
        lineLength++;
      } else {
        lineLength = 40;
      }
    }
  }
}
// 读取上位机发送的单字符指令
// void readCmd() {
//   while (Serial.available() > 0) {
//     char name = Serial.read();
//     if (name != '\r' && name != '\n') armDataCmd(name);
//   }
// }

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
  recordCtrl(); // 录制时按间隔保存更新后的角度
  delay(DSD); // DSD 是机械臂整体运行的等待时间
}
