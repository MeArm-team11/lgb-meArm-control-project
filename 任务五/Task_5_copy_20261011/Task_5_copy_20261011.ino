/*
任务五：meArm机械臂示教绘图程序
  在任务三舵机、摇杆和串口控制基础上，
  根据任务四正逆运动学实现坐标控制、轨迹绘制和五点示教。
功能：
  | O | 张开夹爪 |
  | S | 关闭夹爪 |
  | H | 提高运动速度 |
  | L | 降低运动速度 |
  | P | 输出当前舵机状态 |
  | A | 执行 A 组取放动作 |
  | B | 执行 B 组取放动作 |
  | C | 执行 C 组取放动作 |
  | M | 机械臂回中 |
  | N | 依次选择 A、B、C（目前只输出提示，不执行动作） |
  | R | 开始或停止录制动作 |
  | T | 回放录制动作 |
  发送 x90,y60,z100 并以回车或换行结束，可分别设置底座、前臂、后臂的目标角度。

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
#include "types.h"

// b底座，f前臂，r后臂，c夹爪
Servo base, fArm, rArm, claw;

// 结构体定义已移至 types.h

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
const int cMin = 5;   const int cMax = 110;  // c夹爪

//夹爪开合角度，实物方向相反时交换 cOpen 和 cClose
const int cOpen = cMin;
const int cClose = cMax;  

//上电后的初始角度
int bPos = 90;
int fPos = 90;
int rPos = 90;
int cPos = cMax;

// 运动学参数
// 单位：毫米。当前为任务四文档的名义值，实测校正后再替换
const float L1  = 80.0;   // 大臂轴心距
const float L2  = 80.0;   // 小臂轴心距
const float L3a = 60.0;   // 底座回转轴 -> 肩轴，水平距离
const float L3b = 15.0;   // 腕轴 -> 夹持参考点，水平距离
const float h   = 65.0;   // 肩轴相对安装面的高度

// 逆解算出的角度超出这个差值，视为不可达
const float ARM_EPS = 0.001;

// 把值限制到 [-1, 1]，供 acos() 使用
// 浮点误差可能算出 1.0000001，acos() 会返回 nan
float clampUnit(float value) {
  if (value > 1.0) return 1.0;
  if (value < -1.0) return -1.0;
  return value;
}

// ==================== 正解 FK ====================
// 输入三个数学角度（度）：theta 底座回转角，alpha 大臂倾角，beta 小臂相对角
// 输出夹持参考点的坐标（毫米）
void forwardKinematics(float theta, float alpha, float beta,
                       float &X, float &Y, float &Z) {
  float thetaRad = radians(theta);
  float alphaRad = radians(alpha);
  float betaRad  = radians(beta);

  // R = L3a + L1*cos(alpha) + L2*sin(alpha+beta) + L3b
  float R = L3a + L1 * cos(alphaRad) + L2 * sin(alphaRad + betaRad) + L3b;

  // X = R*cos(theta)，Y = R*sin(theta)
  X = R * cos(thetaRad);
  Y = R * sin(thetaRad);

  // Z = h + L1*sin(alpha) - L2*cos(alpha+beta)
  Z = h + L1 * sin(alphaRad) - L2 * cos(alphaRad + betaRad);
}

// ==================== 逆解 IK ====================
// 输入目标坐标（毫米），输出三个数学角度（度）
// 返回 true 表示求解成功；返回 false 表示目标不可达
bool inverseKinematics(float X, float Y, float Z,
                       float &theta, float &alpha, float &beta) {
  // R = sqrt(X^2 + Y^2)
  float R = sqrt(X * X + Y * Y);

  // theta = atan2(Y, X)。注意是 atan2 不是 atan，能分四个象限
  theta = degrees(atan2(Y, X));

  // 扣掉固定偏移，化成两杆问题
  float r = R - L3a - L3b;
  float z = Z - h;

  // d = 目标点到肩轴的距离
  float d = sqrt(r * r + z * z);

  // 可达性判定：|L1-L2| <= d <= L1+L2
  // 故意不夹紧到边界，明显不可达就返回失败
  if (d > L1 + L2 + ARM_EPS) return false;
  if (d < fabs(L1 - L2) - ARM_EPS) return false;

  // phi 是目标点方位角，gammaS 是肩部内角，eta 是肘部内角
  float phi    = atan2(z, r);
  float gammaS = acos(clampUnit((L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)));
  float eta    = acos(clampUnit((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2)));

  // alpha = phi + gammaS，beta = eta - 90度
  alpha = degrees(phi + gammaS);
  beta  = degrees(eta) - 90.0;

  return true;
}

// 上电初始角度，也是任务三的回中目标
const int bHome = 90;
const int fHome = 90;
const int rHome = 90;
const int cHome = cMax;

//摇杆参数
const int joyCenter = 512; // 摇杆中心值
const int joyDead = 100;   // 摇杆死区，防止松手时抖动
const int joyStep = 1;     // 每次循环改变的角度

//速度参数：DSD 是等待时间，越小越快
int DSD = 35;
const int DSDmin = 2;    // DSD 最小值，速度最快
const int DSDmax = 60;   // DSD 最大值，速度最慢
const int DSDstep = 5;   //每次调整的数值
// 自动动作每一步结束后的等待时间，实测后调整
const int actionWait = 300;

//任务五新增状态记录
int motionState = 0;  // 0：没有自动运动；1：正在自动运动
// 自动运动的目标角度
int targetB = 90;  // 底座的目标角度
int targetF = 90;  // 前臂舵机的目标角度
int targetR = 90;  // 后臂舵机的目标角度
int targetC = cMax;  // 夹爪的目标角度
// 上一次自动运动步进的时间，单位：毫秒
unsigned long lastMotionStep = 0;
unsigned long lastJoyStep = 0;  // 上一次处理摇杆的时间
// 0：没有回中；1：大小臂回中；2：底座回中；3：夹爪回中
int homeStep = 0;

// 播放流程：0 空闲；1 回起点；2 正在播放
int playStage = 0;
// 回起点走到第几步
int returnStep = 0;
// 下一次要播放第几帧
int playIndex = 0;
// 播放起点的时间戳
unsigned long playBegin = 0;

// 0：还没有开始停顿计时；1：正在停顿计时
int homeWaiting = 0;

unsigned long homeWaitStart = 0;

// 当前正在执行的取放数据
ActionData currentAction;

// 取放流程：0 表示空闲，1~10 表示当前步骤
int actionStep = 0;

// 是否正在进行步骤之间的停顿
int actionWaiting = 0;
unsigned long actionWaitStart = 0;

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

    Serial.println(F("开始录制"));
  } else {
    // 保存停止时的姿态，数组已满时 saveFrame() 不再写入
    saveFrame();
    recording = false;

    Serial.print(F("录制结束，帧数="));
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
  cOpen,           // 夹爪张开角度
  cClose           // 夹爪夹紧角度
};

ActionData actionB = {
  {90, 90, 90},
  {90, 90, 90},
  cOpen,
  cClose 
};

ActionData actionC = {
  {90, 90, 90},
  {90, 90, 90},
  cOpen,
  cClose 
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

// 判断三个舵机的指令角度是否都已到达目标
// 返回 1：全部到达；返回 0：还有没到达的
int jointTargetReached() {
  if (bPos == targetB &&
      fPos == targetF &&
      rPos == targetR &&
      cPos == targetC) {
    return 1;
  }

  return 0;
}

// 输出四个舵机当前状态
void printStatus() {
  Serial.print(F("x"));
  Serial.print(bPos);
  Serial.print(F(",y"));
  Serial.print(fPos);
  Serial.print(F(",z"));
  Serial.println(rPos);
  Serial.print(F("c"));
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
      Serial.println(F("录制容量已满，自动停止"));
      Serial.print(F("帧数="));
      Serial.println(frameCount);
    }
  }
}

// 保存当前四个舵机的设定角度
void saveFrame() {
  // 数组已满，不继续写入
  if (frameCount >= recordMax) return;

  recordData[frameCount].b = bPos;
  recordData[frameCount].f = fPos;
  recordData[frameCount].r = rPos;
  recordData[frameCount].c = cPos;

  frameCount++;
}

// 启动播放：只检查和安排第一步
void playStart() {
  if (recording) {
    Serial.println(F("请先停止录制"));
    return;
  }

  if (frameCount < 2) {
    Serial.println(F("没有足够的录制数据"));
    return;
  }

  if (playStage != 0) {
    Serial.println(F("正在播放，请等待"));
    return;
  }

  // 当前姿态和第一帧不一致，先回起点
  if (bPos != recordData[0].b ||
      fPos != recordData[0].f ||
      rPos != recordData[0].r ||
      cPos != recordData[0].c) {
    Serial.println(F("正在返回录制起点"));
    playStage = 1;
    returnStep = 0;
    startJointTarget(recordData[0].b, fPos, rPos, cPos);
    return;
  }

  // 姿态已经对上，直接播放
  playStage = 2;
  playIndex = 0;
  playBegin = millis();
  Serial.println(F("开始播放"));
}

// 每次循环推进一小步播放流程
void playCtrl() {
  if (playStage == 0) {
    return;
  }

  // ---------- 阶段 1：回到录制起点 ----------
  if (playStage == 1) {
    // 当前这一段还没有走完
    if (motionState != 0) {
      return;
    }

    if (returnStep == 0) {
      returnStep = 1;
      startJointTarget(bPos, recordData[0].f, recordData[0].r, cPos);
      return;
    }

    if (returnStep == 1) {
      returnStep = 2;
      startJointTarget(bPos, fPos, rPos, recordData[0].c);
      return;
    }

    // 三步都走完，进入播放
    playStage = 2;
    playIndex = 0;
      playBegin = millis();
    Serial.println(F("开始播放"));
    return;
  }

  // ---------- 阶段 2：按录制时间播放 ----------
  if (playStage == 2) {
    // 所有帧都播完
    if (playIndex >= frameCount) {
      playStage = 0;
      Serial.println(F("播放指令执行结束"));
      printStatus();
      return;
    }

    // 还没到这一帧的时刻，等下一轮
    // 第 i 帧的时刻 = i * recordStep，不再从帧里读时间戳
    if (millis() - playBegin < (unsigned long)playIndex * recordStep) {
      return;
    }

    // 写入这一帧的角度
    servoWrite('b', recordData[playIndex].b);
    servoWrite('f', recordData[playIndex].f);
    servoWrite('r', recordData[playIndex].r);
    servoWrite('c', recordData[playIndex].c);

    playIndex++;
  }
}

// 设置目标，启动三关节自动运动
void startJointTarget(int bTarget, int fTarget, int rTarget,int cTarget) {
  // 忙碌时不接受新的运动目标
  if (motionState != 0) {
    Serial.println(F("正在运动，请等待"));
    return;
  }

  // 超出范围就拒绝，不擅自改变目标
  if (bTarget < bMin || bTarget > bMax ||
      fTarget < fMin || fTarget > fMax ||
      rTarget < rMin || rTarget > rMax ||
      cTarget < cMin || cTarget > cMax) {
    Serial.println(F("目标角度超出范围"));
    return;
  }

  targetB = bTarget;
  targetF = fTarget;
  targetR = rTarget;
  targetC = cTarget;

  lastMotionStep = millis();
  motionState = 1;

  Serial.println(F("开始自动运动"));
}

// 每次调用，最多让三个关节各向目标靠近 1 度
void stepJointTarget() {
  // 不是自动运动状态，就不处理
  if (motionState != 1) {
    return;
  }

  // 已经到达目标，结束运动
  if (jointTargetReached() == 1) {
    motionState = 0;
    // 单独运动时输出；连续流程由各自的控制函数输出完成提示
    if (homeStep == 0 && actionStep == 0) {
      Serial.println(F("自动运动结束"));
      printStatus();
    }
    return;
  }

  // 间隔不足 DSD 毫秒，本次先不动
  unsigned long now = millis();

  if (now - lastMotionStep < (unsigned long)DSD) {
    return;
  }

  lastMotionStep = now;

  // 底座向目标靠近 1 度
  if (bPos < targetB) {
    servoWrite('b', bPos + 1);
  } else if (bPos > targetB) {
    servoWrite('b', bPos - 1);
  }

  // 前臂向目标靠近 1 度
  if (fPos < targetF) {
    servoWrite('f', fPos + 1);
  } else if (fPos > targetF) {
    servoWrite('f', fPos - 1);
  }

  // 后臂向目标靠近 1 度
  if (rPos < targetR) {
    servoWrite('r', rPos + 1);
  } else if (rPos > targetR) {
    servoWrite('r', rPos - 1);
  }
  // 夹爪向目标靠近 1 度
  if (cPos < targetC) {
    servoWrite('c', cPos + 1);
  } else if (cPos > targetC) {
    servoWrite('c', cPos - 1);
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

// 根据指定物体的数据，完成夹取和放置
// 根据 actionStep，启动对应的一段运动
void startActionStep() {
  switch (actionStep) {
    case 1:
      // 大小臂抬升
      startJointTarget(bPos, fHigh, rHigh, cPos);
      break;

    case 2:
      // 张开夹爪
      startJointTarget(bPos, fPos, rPos, currentAction.open);
      break;

    case 3:
      // 底座转向取物点
      startJointTarget(currentAction.getPoint.b, fPos, rPos, cPos);
      break;

    case 4:
      // 大小臂到达取物点
      startJointTarget(bPos,
                       currentAction.getPoint.f,
                       currentAction.getPoint.r,
                       cPos);
      break;

    case 5:
      // 夹紧物体
      startJointTarget(bPos, fPos, rPos, currentAction.close);
      break;

    case 6:
      // 夹取后抬升
      startJointTarget(bPos, fHigh, rHigh, cPos);
      break;

    case 7:
      // 底座转向放置点
      startJointTarget(currentAction.putPoint.b, fPos, rPos, cPos);
      break;

    case 8:
      // 大小臂到达放置点
      startJointTarget(bPos,
                       currentAction.putPoint.f,
                       currentAction.putPoint.r,
                       cPos);
      break;

    case 9:
      // 张开夹爪，放下物体
      startJointTarget(bPos, fPos, rPos, currentAction.open);
      break;

    case 10:
      // 放置后再次抬升
      startJointTarget(bPos, fHigh, rHigh, cPos);
      break;

    default:
      return;
  }

  // 底层没有接受目标，就结束流程，不跳过失败步骤
  if (motionState != 1) {
    actionStep = 0;
    actionWaiting = 0;
    lastJoyStep = millis();
    Serial.println(F("取放步骤启动失败，流程已停止"));
  }
}
// 保存本次动作数据，启动第一步
void doAction(ActionData data) {
  if (motionState != 0 || homeStep != 0 || actionStep != 0 || playStage != 0) {
    Serial.println(F("正在运动，请等待"));
    return;
  }

  currentAction = data;
  actionStep = 1;
  actionWaiting = 0;

  startActionStep();
}
// 每次循环检查一次取放进度
void actionCtrl() {
  if (actionStep == 0) {
    return;
  }

  // 当前这一段还没有结束
  if (motionState != 0) {
    return;
  }

  // 当前这一段结束，开始停顿计时
  if (actionWaiting == 0) {
    actionWaitStart = millis();
    actionWaiting = 1;
    return;
  }

  // 停顿时间还没有到
  if (millis() - actionWaitStart < (unsigned long)actionWait) {
    return;
  }

  actionWaiting = 0;

  // 最后一步及停顿结束
  if (actionStep == 10) {
    actionStep = 0;
    lastJoyStep = millis();

    Serial.println(F("取放动作执行结束"));
    printStatus();
    return;
  }

  // 开始下一步
  actionStep++;
  startActionStep();
}

// 下次按键1要执行的物体
char nextObject = 'A';

// 每调用一次，依次选择 A、B、C
void nextAction() {
  switch (nextObject) {
    case 'A':
      Serial.println(F("按键1选择A"));
      // doAction(actionA); // 待A完成标定后启用
      nextObject = 'B';
      break;

    case 'B':
      Serial.println(F("按键1选择B"));
      // doAction(actionB); // 待B完成标定后启用
      nextObject = 'C';
      break;

    case 'C':
      Serial.println(F("按键1选择C"));
      // doAction(actionC); // 待C完成标定后启用
      nextObject = 'A';
      break;
  }
}

// 启动回中流程，只安排第一步
void goHome() {
  // 防止正在运动时重复启动回中
  if (motionState != 0 || homeStep != 0 || actionStep != 0) {
    Serial.println(F("正在运动，请等待"));
    return;
  }

  homeStep = 1;
  homeWaiting = 0;

  Serial.println(F("开始回中"));

  // 第一步：大小臂直接回中，底座和夹爪保持不动
  startJointTarget(bPos, fHome, rHome, cPos);
}

// 检查回中进度，每次只处理一小步
void homeCtrl() {
  if (homeStep == 0) {
    return;
  }

  // 当前这段运动还没有结束
  if (motionState != 0) {
    return;
  }

  // 当前这段结束，开始停顿计时
  if (homeWaiting == 0) {
    homeWaitStart = millis();
    homeWaiting = 1;
    return;
  }

  // 停顿时间未到
  if (millis() - homeWaitStart < (unsigned long)actionWait) {
    return;
  }

  homeWaiting = 0;

    switch (homeStep) {
    case 1:
      // 大小臂回中完成，开始底座回中
      homeStep = 2;
      startJointTarget(bHome, fPos, rPos, cPos);
      break;

    case 2:
      // 底座回中完成，开始夹爪回中
      homeStep = 3;
      startJointTarget(bPos, fPos, rPos, cHome);
      break;

    case 3:
      // 三步及最后的停顿全部完成
      homeStep = 0;
      lastJoyStep = millis();

      Serial.println(F("回中指令执行结束"));
      printStatus();
      break;
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
  if ((motionState != 0 || homeStep != 0 || actionStep != 0 || playStage != 0) &&
      name != 'P' &&
      name != 'H' &&
      name != 'L') {
    Serial.println(F("正在自动运动，暂不接受此指令"));
    return;
  }
  switch (name) {
    case 'O':
      startJointTarget(bPos, fPos, rPos, cOpen);
      break;

    case 'S':
      startJointTarget(bPos, fPos, rPos, cClose);
      break;
    
    case 'P':
      printStatus();//输出状态
      break;

    case 'H':
      DSD = DSD - DSDstep;
      if (DSD < DSDmin) DSD = DSDmin;
      Serial.print(F("提高速度，DSD="));
      Serial.println(DSD);
      break;

    case 'L':
      DSD = DSD + DSDstep;
      if (DSD > DSDmax) DSD = DSDmax;
      Serial.print(F("降低速度，DSD="));
      Serial.println(DSD);
      break;
      
    case 'A':
      Serial.println(F("开始执行A"));
      doAction(actionA);
      break;

    case 'B':
      Serial.println(F("开始执行B"));
      doAction(actionB);
      break;

    case 'C':
      Serial.println(F("开始执行C"));
      doAction(actionC);
      break;

    case 'M':
      goHome();
      break;

    case 'N':
      nextAction();
      break;

    case 'R':
      recordCmd();
      break;
    
    case 'T':
      playStart();
      break;

    case 'K':
      kinematicsTest();
      break;

    default:
      Serial.println(F("无法识别指令"));
      break;
  }
}

// 解析三个舵机的目标角度，启动非阻塞运动
void armXYZCmd() {
  if (homeStep != 0 || actionStep != 0) {
    Serial.println(F("正在执行自动流程，暂不接受角度指令"));
    return;
  }
  int x, y, z;
  char extra;

  int count = sscanf(
    serialLine,
    " x%d , y%d , z%d %c",
    &x, &y, &z, &extra
  );

  if (count == 3) {
    startJointTarget(x, y, z, cPos);
  } else {
    Serial.println(F("指令格式错误"));
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
        Serial.println(F("指令太长"));
      }
      lineLength = 0;
    } else {
      char name = data;
      if (name >= 'a' && name <= 'z') name = name - 'a' + 'A';

      if (name == 'O' || name == 'S' || name == 'H' || name == 'L' || 
          name == 'P' || name == 'A' || name == 'B' || name == 'C' || 
          name == 'M' || name == 'N' || name == 'R' || name == 'T' ||
          name == 'K') {
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

// ==================== 运动学自检：往返测试 ====================
// 原理：正解和逆解互为逆运算。
// 取一组角度 -> 正解算坐标 -> 逆解算回角度 -> 应该等于原来那组
void kinematicsTest() {
  // 三组测试角度，都在可达范围内
  float testTheta[3] = {  0.0, 30.0,  0.0};
  float testAlpha[3] = { 60.0, 60.0, 90.0};
  float testBeta[3]  = {-20.0,-20.0,-60.0};

  Serial.println(F("===== 运动学往返测试 ====="));

  for (int i = 0; i < 3; i++) {
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    // 正解
    forwardKinematics(testTheta[i], testAlpha[i], testBeta[i], X, Y, Z);

    // 逆解
    float theta2 = 0.0;
    float alpha2 = 0.0;
    float beta2  = 0.0;
    bool ok = inverseKinematics(X, Y, Z, theta2, alpha2, beta2);

    Serial.print(F("第"));
    Serial.print(i + 1);
    Serial.println(F("组"));

    Serial.print(F("  输入角度  theta="));
    Serial.print(testTheta[i], 2);
    Serial.print(F("  alpha="));
    Serial.print(testAlpha[i], 2);
    Serial.print(F("  beta="));
    Serial.println(testBeta[i], 2);

    Serial.print(F("  正解坐标  X="));
    Serial.print(X, 2);
    Serial.print(F("  Y="));
    Serial.print(Y, 2);
    Serial.print(F("  Z="));
    Serial.println(Z, 2);

    if (!ok) {
      Serial.println(F("  逆解失败：目标不可达"));
      continue;
    }

    Serial.print(F("  逆解角度  theta="));
    Serial.print(theta2, 2);
    Serial.print(F("  alpha="));
    Serial.print(alpha2, 2);
    Serial.print(F("  beta="));
    Serial.println(beta2, 2);

    Serial.print(F("  误差      theta="));
    Serial.print(theta2 - testTheta[i], 3);
    Serial.print(F("  alpha="));
    Serial.print(alpha2 - testAlpha[i], 3);
    Serial.print(F("  beta="));
    Serial.println(beta2 - testBeta[i], 3);
  }

  Serial.println(F("===== 测试结束 ====="));
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
  Serial.println(F("欢迎使用"));

  // 机械臂上电先回到设定的初始角度
  servoWrite('b', bPos);
  servoWrite('f', fPos);
  servoWrite('r', rPos);
  servoWrite('c', cPos);
}

void loop() {
  readCmd();
  stepJointTarget();
  homeCtrl();
  actionCtrl();
  playCtrl();

  // 所有自动流程都空闲，才允许摇杆控制
  if (motionState == 0 && homeStep == 0 && actionStep == 0 && playStage == 0) {
    unsigned long now = millis();
    if (now - lastJoyStep >= (unsigned long)DSD) {
      lastJoyStep = now;
      joyCtrl();
    }
  } else {
    lastJoyStep = millis();
  }
  recordCtrl();
}