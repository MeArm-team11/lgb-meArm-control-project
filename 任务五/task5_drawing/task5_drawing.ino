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
  | N | 依次选择 A、B、C 并执行取放 |
  | R | 开始或停止录制动作 |
  | T | 回放录制动作 |
  | K | 正逆运动学往返自检 |
  发送 x90,y60,z100 并以回车或换行结束，可分别设置底座、前臂、后臂的目标角度。

按键控制板（Uno R3 + IIC OLED，8 键）信号：
  | 按键 | 引脚 | 字符 | 功能 |
  | 1 | D4  | N | 循环执行 A/B/C 夹取放置 |
  | 2 | D5  | R | 开始 / 停止录制 |
  | 3 | D6  | T | 回放上次录制的动作 |
  | 4 | D7  | M | 回到初始姿态 |
  | 5 | D8  | F | 绘图模式循环切换 |
  | 6 | D9  | D | 开始绘制 |
  | 7 | D10 | I | 记录当前示教点 |
  | 8 | D11 | Q | 取消并回到初始姿态 |
  按钮板 TX(D3) 接本板 D12(RX)，两板必须共地，波特率 9600，只发单字符。

摇杆标定（任务二）：
  用摇杆把夹爪移到目标位置后从上位机发送数字键记录当前三个关节角
  | 1 | A 取物点 |  | 2 | A 放置点 |
  | 3 | B 取物点 |  | 4 | B 放置点 |
  | 5 | C 取物点 |  | 6 | C 放置点 |
  标定数据存在 SRAM，掉电或复位后需要重新标定。

任务二取放流程（startActionStep，共 14 步）：
   1  抬臂到安全姿态          f=120, r=85
   2  张开夹爪                c=5
   3  底座转向取物点          只动底座
   4  到"取物点正上方"的接近姿态  f = 取物点.f - approachLift
   5  【垂直下降】到取物点      只动大臂
   6  夹紧物体
   7  【垂直抬大臂】提离桌面    只动大臂
   8  【收小臂】               f=fFold, r=rFold
   9  底座转向放置点          只动底座
  10  到"放置点正上方"的接近姿态
  11  【垂直下降】到放置点      只动大臂
  12  张开夹爪，放下物体
  13  【垂直抬大臂】离开物体    只动大臂
  14  【收小臂】回到安全姿态

  三个关键设计：
    1) 先抬臂再转底座：手臂低位回转会横扫桌面和其他物体，
       先抬到安全姿态再转，旋转路径才安全。
    2) 接近姿态 + 垂直下降：第 4、10 步先把夹爪停在目标点正上方，
       第 5、11 步只动大臂垂直落下。若从安全姿态斜着移到取物点，
       爪尖会从斜上方扫过物体、把物体推跑（早期版本夹不住的主因）。
    3) 夹住后先抬大臂、再收小臂：大臂每度约抬 1.4mm，抬升力臂最短、
       对夹爪拉力最小，物体最不容易被拽掉；而且物体先垂直离桌，
       收小臂时不会贴着桌面拖过去。放置时（13、14 步）同理。

  可调量：approachLift（接近高度，默认 22 度约 30mm）、
          liftAmount（抬大臂高度，默认 35 度约 49mm）、
          fFold / rFold（收臂后的姿态，默认 110 / 60）。

任务五画线流程（drawStart / drawCtrl）：
  操作步骤：
    1. 记示教点：用摇杆把笔尖移到直线起点，按 I（按键7）记一个点；
       再移到终点，按 I 记第二个点。记满 5 点后再按 I 会清空重记。
    2. 按 D（按键6）开始绘制。
    3. 绘制过程中按 D 暂停，再按 D 继续；按 Q（按键8）取消并回待机。
       串口发 U 也可以直接继续（备用）。
  自动流程：
    先转到底座起点角度并抬臂 -> 降到起笔点 -> 在相邻示教点之间
    插值走线 -> 走完所有点后收笔。
  插值方式：
    按角度差把每一段分成若干步，底座、大臂、小臂同步按比例走。
    因为起点和终点都是"笔尖落在纸面上"的姿态，中间按比例插值时
    笔尖高度基本不变，所以画出来的线是连续的。
    角度差的线性插值会让线略有弧度，一段几十毫米的线误差在几毫米以内。
  可调量：drawStepMs（每个插值点之间的间隔，默认 15ms，越小越快）。

接线：
  底座舵机 D9，前臂舵机 D8，后臂舵机 D6，夹爪舵机 D7。
  底座摇杆 A0，前臂摇杆 A3，后臂摇杆 A1，夹爪摇杆 A2。

串口波特率：9600。
x,y,z 指令以回车或换行结束。
DSD 是每移动 1 度后的等待时间，DSD 越小，运行越快。

使用前应核对舵机转向、机械限位和夹爪开合角度。

还未实现功能：
  1. 绘制过程中的暂停 / 继续（任务五子任务 3）
  2. 按 F 切换后的绘图模式只改变显示名称，尚未按模式绘制不同轨迹
  3. 平滑曲线的插值算法（任务五子任务 5）
*/

#include <Servo.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include "types.h"

// b底座，f前臂，r后臂，c夹爪
Servo base, fArm, rArm, claw;

// ==================== 按钮板通信 ====================
// 按钮板（Uno R3 + IIC OLED，8 键）通过软串口发单字符指令
// 按钮板 TX(D3) -> 机械臂 D12(RX)
// 与电脑用的硬件串口 D0/D1 分开，两块板烧录都不需要拔线
const int BTN_RX = 12;
const int BTN_TX = 13;
SoftwareSerial btnSerial(BTN_RX, BTN_TX);

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
const int bMin = 10;   const int bMax = 170; // b底座
const int fMin = 10;  const int fMax = 170; // f前臂15 120
const int rMin = 10;  const int rMax = 170; // r后臂
const int cMin = 5;   const int cMax = 130;  // c夹爪

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

// 取放流程：0 表示空闲，1~14 表示当前步骤
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

// ==================== 任务二：摇杆标定 ====================
// 操作流程：用摇杆把夹爪移到目标位置 -> 从上位机发一个数字键记录当前角度
//   1 = A 取物点   2 = A 放置点
//   3 = B 取物点   4 = B 放置点
//   5 = C 取物点   6 = C 放置点
// 注意：标定数据存在 SRAM，掉电或复位后回到默认值，需要重新标定
bool actionReady[3] = {false, false, false};

// 把三个动作数据放进数组，标定时按下标访问
ActionData* actionList[3] = {&actionA, &actionB, &actionC};

// 记录当前姿态到指定物体的指定点
// objIndex: 0=A 1=B 2=C    isGetPoint: true=取物点 false=放置点
void calibratePoint(int objIndex, bool isGetPoint) {
  JointPoint* p;

  if (isGetPoint) {
    p = &actionList[objIndex]->getPoint;
  } else {
    p = &actionList[objIndex]->putPoint;
  }

  // 记录的当前舵机设定角度，就是标定结果
  p->b = bPos;
  p->f = fPos;
  p->r = rPos;

  actionReady[objIndex] = true;

  Serial.print(F("标定 OK: "));
  Serial.print((char)('A' + objIndex));
  Serial.print(isGetPoint ? F(" 取物点") : F(" 放置点"));
  Serial.print(F("  b="));
  Serial.print(p->b);
  Serial.print(F(" f="));
  Serial.print(p->f);
  Serial.print(F(" r="));
  Serial.println(p->r);
}

// ==================== 任务五：示教与绘制 ====================
// 记录的是关节角而不是末端坐标，所以绘制不需要逆解，
// 路径上的点都在关节空间里算，笔尖高度基本不变
const int teachMax = 5;
JointPoint teachPoint[teachMax];   // 最多 5 个示教点
int teachCount = 0;                // 已记录几个点

// 绘制模式
//   0 LINE  ：2 个点画直线（开放折线）
//   1 TRI   ：3 个点画三角形（自动闭合，不用把第 1 点再记一遍）
//   2 POLY  ：5 个点画折线（任务书子任务 4，开放）
//   3 CURVE ：3~5 个点画平滑曲线（Catmull-Rom，子任务 5）
const int MODE_LINE  = 0;
const int MODE_TRI   = 1;
const int MODE_POLY  = 2;
const int MODE_CURVE = 3;
const int MODE_COUNT = 4;
int drawMode = MODE_LINE;

// 绘制时每个路径点之间的间隔（毫秒）
// 和取放流程用的 DSD 分开：绘制需要更密集地更新目标位置，
// 间隔越小笔迹越连续；太小会给舵机堆太多目标。
const unsigned long drawStepMs = 15;

// 预生成的路径。按 D 的时候一次性把整条路径算好存进来，
// 绘制过程中只按顺序取点，不在运动中间做浮点运算。
//
// 容量要按最长的模式算：曲线最多 5 个点 = 4 段，每段采样 18 个点，
// 一共 72 个点。取 120 留足余量。
// 每个点 3 个 int = 6 字节，120 个 = 720 字节。
// 不能再大了：Uno 只有 2KB SRAM，之前就因为内存不足出现过动作乱执行。
const int PATH_MAX = 120;
struct PathPoint {
  int b;
  int f;
  int r;
};
PathPoint pathBuf[PATH_MAX];
int pathLen = 0;      // 路径上实际有多少个点

// 绘制流程的状态
// drawActing: 0=空闲；1=正在回起点；2=正在绘制；3=正在取消（先停住再回待机）；4=已暂停
int drawActing = 0;
int drawStage = 0;    // 回起点的两个子步骤：0 抬臂转底座、1 降到起点
int pathIndex = 0;    // 当前走到路径的第几个点
unsigned long lastDrawStep = 0;   // 上一次推进路径的时间
unsigned long drawWaitStart = 0;   // 子步骤之间的停顿计时

// 把 (x, y) 限幅后返回
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Catmull-Rom 样条插值：返回 p1 到 p2 之间 t（0~1）处的值
// p0 和 p3 是相邻点，用它们来定两端的切线方向。
// 相邻两段的切线自动一致，所以曲线没有尖角；
// 而且它"经过"所有给定点，是插值不是逼近（贝塞尔不经过控制点，不能用）。
//
// 三次形式展开后  t=0 时结果等于 p1、t=1 时等于 p2，
// 所以在端点把 p0 夹成 p1、p3 夹成 p2，曲线就不会冲出端点外。
float catmullRom(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5 * (
      2.0 * p1
    + (-p0 + p2) * t
    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

// 生成折线路径：相邻点之间线性采样
// closed = true 时自动闭合（三角形的第 3 段就是 点3 -> 点1）
void buildLinePath(bool closed) {
  pathLen = 0;

  int segCount = teachCount - 1;
  if (closed) {
    segCount = teachCount;   // 多走一段回到起点
  }

  for (int s = 0; s < segCount; s++) {
    int i0 = s;
    int i1 = (s + 1) % teachCount;   // 取模实现闭合

    // 每段采样多少个点：按角度差算，最少 12 个
    int maxDiff = abs(teachPoint[i1].b - teachPoint[i0].b);
    if (abs(teachPoint[i1].f - teachPoint[i0].f) > maxDiff) {
      maxDiff = abs(teachPoint[i1].f - teachPoint[i0].f);
    }
    if (abs(teachPoint[i1].r - teachPoint[i0].r) > maxDiff) {
      maxDiff = abs(teachPoint[i1].r - teachPoint[i0].r);
    }
    int samples = maxDiff;
    if (samples < 12) samples = 12;

    for (int k = 1; k <= samples; k++) {
      if (pathLen >= PATH_MAX) return;
      float t = (float)k / (float)samples;
      pathBuf[pathLen].b = teachPoint[i0].b + (int)((teachPoint[i1].b - teachPoint[i0].b) * t + 0.5);
      pathBuf[pathLen].f = teachPoint[i0].f + (int)((teachPoint[i1].f - teachPoint[i0].f) * t + 0.5);
      pathBuf[pathLen].r = teachPoint[i0].r + (int)((teachPoint[i1].r - teachPoint[i0].r) * t + 0.5);
      pathLen++;
    }
  }
}

// 生成平滑曲线路径（Catmull-Rom）
// closed = true 时首尾也连起来，形成闭合的平滑轮廓
void buildCurvePath(bool closed) {
  pathLen = 0;

  int segCount = teachCount - 1;
  if (closed) {
    segCount = teachCount;
  }

  for (int s = 0; s < segCount; s++) {
    int i1 = s;
    int i2 = (s + 1) % teachCount;

    // 取相邻点作为切线参考；端点处夹回自身，避免曲线冲出范围
    int i0 = i1 - 1;
    int i3 = i2 + 1;
    if (closed) {
      i0 = (i1 - 1 + teachCount) % teachCount;
      i3 = (i2 + 1) % teachCount;
    } else {
      if (i0 < 0) i0 = i1;
      if (i3 >= teachCount) i3 = i2;
    }

    int samples = 18;   // 曲线每段固定采样 18 个点（5 点 4 段 = 72 点，在 PATH_MAX 内）

    for (int k = 1; k <= samples; k++) {
      if (pathLen >= PATH_MAX) return;
      float t = (float)k / (float)samples;
      pathBuf[pathLen].b = (int)clampf(
          catmullRom(teachPoint[i0].b, teachPoint[i1].b, teachPoint[i2].b, teachPoint[i3].b, t), bMin, bMax);
      pathBuf[pathLen].f = (int)clampf(
          catmullRom(teachPoint[i0].f, teachPoint[i1].f, teachPoint[i2].f, teachPoint[i3].f, t), fMin, fMax);
      pathBuf[pathLen].r = (int)clampf(
          catmullRom(teachPoint[i0].r, teachPoint[i1].r, teachPoint[i2].r, teachPoint[i3].r, t), rMin, rMax);
      pathLen++;
    }
  }
}

// 显示和日志用的模式名
const char* modeName(int m) {
  switch (m) {
    case MODE_LINE:  return "LINE";
    case MODE_TRI:   return "TRI";
    case MODE_POLY:  return "POLY";
    default:         return "CURVE";
  }
}

// 记录当前姿态为一个示教点
// 已满 5 点时再按 = 清空并从第 1 点重新开始（现场记错点的补救手段）
void teachRecordPoint() {
  if (teachCount >= teachMax) {
    teachCount = 1;
    Serial.println(F("示教点已满，清空重记：现在记的是第 1 点"));
  } else {
    teachCount++;
  }

  teachPoint[teachCount - 1].b = bPos;
  teachPoint[teachCount - 1].f = fPos;
  teachPoint[teachCount - 1].r = rPos;

  Serial.print(F("记录示教点 "));
  Serial.print(teachCount);
  Serial.print(F("/"));
  Serial.print(teachMax);
  Serial.print(F("  b="));
  Serial.print(bPos);
  Serial.print(F(" f="));
  Serial.print(fPos);
  Serial.print(F(" r="));
  Serial.println(rPos);
}

// 让夹爪走到指定位置，只动大臂和小臂（底座、夹爪不动）
// 底座在"回起点"的第一步就转到位了，之后一直不转，
// 所以画笔只在手臂平面内移动，不会因为转底座而在纸上划出弧线
void startArmMove(int fTarget, int rTarget) {
  startJointTarget(bPos, fTarget, rTarget, cPos);
}

// 判断大臂、小臂是否都已经走到目标（底座和夹爪在绘制期间不动，不用判）
int armReached() {
  if (fPos == targetF && rPos == targetR) {
    return 1;
  }
  return 0;
}

// 启动绘制：先转到起点的底座角度，再降到起点，然后插值走线
void drawStart() {
  if (teachCount < 2) {
    Serial.print(F("示教点不足，当前 "));
    Serial.print(teachCount);
    Serial.println(F(" 个（至少需要 2 个）"));
    return;
  }

  // 正在绘制时，同一个命令兼作"暂停"
  if (drawActing == 2) {
    drawActing = 4;
    Serial.println(F("绘制已暂停（再按一次继续，按 Q 取消）"));
    return;
  }

  // 已经暂停时，同一个命令兼作"继续"
  if (drawActing == 4) {
    drawActing = 2;
    Serial.println(F("继续绘制"));
    return;
  }

  // 正在回起点、或正在取消过程中，不接受新请求
  if (drawActing != 0) {
    Serial.println(F("正在执行绘制流程，请等待"));
    return;
  }

  // 忙碌时不接受新的绘制请求
  if (motionState != 0 || homeStep != 0 || actionStep != 0 || playStage != 0) {
    Serial.println(F("正在运动，请等待"));
    return;
  }

  Serial.print(F("开始绘制，模式 "));
  Serial.print(modeName(drawMode));
  Serial.print(F("，共 "));
  Serial.print(teachCount);
  Serial.println(F(" 个点"));

  // 按模式校验点数：三角形必须要 3 个，曲线至少 3 个
  if (drawMode == MODE_TRI && teachCount != 3) {
    Serial.println(F("三角形模式需要正好 3 个示教点"));
    return;
  }
  if (drawMode == MODE_CURVE && teachCount < 3) {
    Serial.println(F("平滑曲线至少需要 3 个示教点"));
    return;
  }

  // 按模式生成整条路径，存在 pathBuf 里。
  // 放在这里一次算完，绘制过程中就不做浮点运算了。
  if (drawMode == MODE_CURVE) {
    buildCurvePath(false);      // 平滑曲线：不闭合，经过所有点
  } else if (drawMode == MODE_TRI) {
    buildLinePath(true);        // 三角形：折线，自动闭合
  } else {
    buildLinePath(false);       // 直线 / 五点折线：折线，开放
  }

  if (pathLen < 2) {
    Serial.println(F("路径生成失败，请检查示教点"));
    return;
  }

  Serial.print(F("路径点数 "));
  Serial.println(pathLen);

  // 先把底座转到第一个点的方向，同时把大小臂抬到安全姿态。
  // 为什么要先抬臂：起笔点在纸面上，直接从落笔姿态转底座会拖到纸面。
  drawActing = 1;
  drawStage = 0;
  startJointTarget(teachPoint[0].b, fHigh, rHigh, cPos);
}

// 取消绘制：把状态置为"正在取消"，由 drawCtrl 等当前这一段停下后再回待机
// 不在按键处理里直接调 goHome()，是因为绘制中 motionState 还是 1，
// goHome() 会以"正在运动，请等待"为由拒绝执行，取消就失效了
void drawCancel() {
  if (drawActing == 0) {
    Serial.println(F("当前没有在绘制"));
    return;
  }

  if (drawActing == 3) {
    Serial.println(F("已经在取消中"));
    return;
  }

  drawActing = 3;
  Serial.println(F("收到取消指令，等当前动作结束后回待机"));
}

// 绘制推进：每轮 loop() 调用一次，没有到节拍就直接返回
void drawCtrl() {
  if (drawActing == 0) {
    return;
  }

  // 当前这一段运动还没有走完，等它走完再说
  if (motionState != 0) {
    return;
  }

  // ---------- 取消：等当前这一段停下，再回待机 ----------
  // 不直接把正在进行的运动打断，避免舵机突然反向或丢步
  if (drawActing == 3) {
    drawActing = 0;
    Serial.println(F("已取消绘制，回到待机姿态"));
    goHome();
    return;
  }

  // ---------- 暂停：什么都不推进，进度保留 ----------
  // pathIndex 不动，所以"继续"时从原地接着画
  if (drawActing == 4) {
    return;
  }

  // ---------- 阶段 1：先回到起笔点 ----------
  if (drawActing == 1) {
    if (drawStage == 0) {
      // 底座已经转到起点方向、大小臂也抬起来了，下一步降到起笔点
      drawStage = 1;
      startArmMove(teachPoint[0].f, teachPoint[0].r);
      return;
    }

    // 已经降到起笔点，进入绘制
    drawActing = 2;
    pathIndex = 0;
    lastDrawStep = millis();
    Serial.println(F("开始画线"));
    return;
  }

  // ---------- 阶段 2：沿预生成的路径走点 ----------
  // LINE / TRI / POLY / CURVE 的区别只在"路径怎么生成"，
  // 生成之后都走同一段代码，所以暂停、继续、取消对所有模式都通用
  if (drawActing == 2) {
    if (pathIndex >= pathLen) {
      drawActing = 0;
      Serial.println(F("绘制完成"));
      printStatus();
      return;
    }

    // 等上一个路径点走到底
    if (armReached() == 0) {
      return;
    }

    // 等一个节拍，让舵机有时间真正稳定下来
    if (millis() - lastDrawStep < drawStepMs) {
      return;
    }
    lastDrawStep = millis();

    startJointTarget(pathBuf[pathIndex].b,
                     pathBuf[pathIndex].f,
                     pathBuf[pathIndex].r,
                     cPos);
    pathIndex++;

    // 每走 20 个点报一次进度，方便现场判断画到哪了
    if (pathIndex % 20 == 0) {
      Serial.print(F("进度 "));
      Serial.print(pathIndex);
      Serial.print(F("/"));
      Serial.println(pathLen);
    }
    return;
  }
}

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

// 接近姿态的抬升量（度）
// 抓取前先到物体"正上方"的接近姿态，再垂直下降。
// 大臂角度每减小 1 度，夹爪大约抬高 1.4mm（L1 = 80mm）：
//   f = 取物点.f - 22  ->  夹爪比取物点高约 30mm
// 这个高度要能越过物体本身和旁边的物体，又不能太高（抬得越高，
// 抬升那一步的行程越长，物体越容易被甩出去）。
//
// 不能写成 const：接近姿态的角度要经过 limitData() 限幅再使用，
// 写成 const 会让编译器在初始化时调用函数，把数据段搬到 RAM，
// 白白占用本就紧张的 SRAM。
int approachLift = 22;

// 夹住物体之后的抬升分两步做：先抬大臂，再收小臂
// 为什么分开：
//   1) 大臂每减小 1 度，夹爪约抬高 1.4mm，抬升的力臂最短，
//      对夹爪的拉力最小，物体最不容易被甩出去；
//   2) 先让物体垂直离开桌面，再往回收小臂，
//      物体不会贴着桌面拖过去、蹭到旁边的物体。
// 大臂先抬的高度（度），抬到"取物点.f - 这个值"
int liftAmount = 35;
// 再把小臂收到这个姿态（这两步收完后回到安全姿态附近）
int fFold = 110;
int rFold = 60;

// 取放流程：1~14 共 14 步
// 结构参考通用取放标准做法：每个位置分"接近姿态"和"就位姿态"两次，
// 中间靠垂直下降/上升连接，避免斜着扫过去把物体推跑。
//   1  抬臂到安全姿态
//   2  张开夹爪
//   3  底座转向取物点
//   4  大小臂到"取物点正上方"的接近姿态
//   5  【垂直下降】到取物点（只动大臂，底座和小臂不动）
//   6  夹紧物体
//   7  【抬大臂】把物体垂直提离桌面
//   8  【收小臂】把手臂收回（物体已经在高处，不会再蹭到桌面）
//   9  底座转向放置点
//  10  大小臂到"放置点正上方"的接近姿态
//  11  【垂直下降】到放置点（只动大臂）
//  12  张开夹爪，放下物体
//  13  抬大臂，离开物体
//  14  收小臂，回到安全姿态
void startActionStep() {
  switch (actionStep) {
    case 1:
      // 大小臂抬升到安全姿态
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
      // 大小臂到"取物点正上方"的接近姿态
      // 只抬大臂（f 减小 = 抬高），底座和小臂在下一步下降时保持不动
      // 用 limitData 限幅：取物点的 f 如果接近上限，减 22 之后可能越界，
      // 而越界的目标会被 startJointTarget 拒绝并导致整个流程中止
      startJointTarget(bPos,
                       limitData(currentAction.getPoint.f - approachLift, fMin, fMax),
                       currentAction.getPoint.r,
                       cPos);
      break;

    case 5:
      // 【垂直下降】到取物点：只动大臂
      // 底座和小臂不动，所以夹爪在铅垂方向落下去，
      // 不会从斜上方扫过来把物体推跑
      startJointTarget(bPos,
                       currentAction.getPoint.f,
                       rPos,
                       cPos);
      break;

    case 6:
      // 夹紧物体
      startJointTarget(bPos, fPos, rPos, currentAction.close);
      break;

    case 7:
      // 【抬大臂】先把物体垂直提离桌面
      // 只动大臂，小臂保持不动，所以是垂直提起来
      startJointTarget(bPos,
                       limitData(currentAction.getPoint.f - liftAmount, fMin, fMax),
                       rPos,
                       cPos);
      break;

    case 8:
      // 【收小臂】物体已经离桌，再往回收手臂
      startJointTarget(bPos, fFold, rFold, cPos);
      break;

    case 9:
      // 底座转向放置点
      startJointTarget(currentAction.putPoint.b, fPos, rPos, cPos);
      break;

    case 10:
      // 大小臂到"放置点正上方"的接近姿态
      startJointTarget(bPos,
                       limitData(currentAction.putPoint.f - approachLift, fMin, fMax),
                       currentAction.putPoint.r,
                       cPos);
      break;

    case 11:
      // 【垂直下降】到放置点：只动大臂
      startJointTarget(bPos,
                       currentAction.putPoint.f,
                       rPos,
                       cPos);
      break;

    case 12:
      // 张开夹爪，放下物体
      startJointTarget(bPos, fPos, rPos, currentAction.open);
      break;

    case 13:
      // 抬大臂，离开刚放下的物体
      startJointTarget(bPos,
                       limitData(currentAction.putPoint.f - liftAmount, fMin, fMax),
                       rPos,
                       cPos);
      break;

    case 14:
      // 收小臂，回到安全姿态
      startJointTarget(bPos, fFold, rFold, cPos);
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

  // 最后一步及停顿结束（现在流程共 14 步）
  if (actionStep == 14) {
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

// 每调用一次，依次选择 A、B、C，并执行对应的取放动作
void nextAction() {
  switch (nextObject) {
    case 'A':
      Serial.println(F("按键1选择A"));
      doAction(actionA);
      nextObject = 'B';
      break;

    case 'B':
      Serial.println(F("按键1选择B"));
      doAction(actionB);
      nextObject = 'C';
      break;

    case 'C':
      Serial.println(F("按键1选择C"));
      doAction(actionC);
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

    case 'K':
      kinematicsTest();
      break;

    // ===== 任务二：摇杆标定（用摇杆摆好位置后发送数字键）=====
    case '1': calibratePoint(0, true);  break;  // A 取物点
    case '2': calibratePoint(0, false); break;  // A 放置点
    case '3': calibratePoint(1, true);  break;  // B 取物点
    case '4': calibratePoint(1, false); break;  // B 放置点
    case '5': calibratePoint(2, true);  break;  // C 取物点
    case '6': calibratePoint(2, false); break;  // C 放置点

    // ===== 按钮板按键 1~4 =====
    case 'N': nextAction();   break;  // 按键1：依次选择 A、B、C 并执行取放
    case 'R': recordCmd();    break;  // 按键2：开始 / 停止录制
    case 'T': playStart();    break;  // 按键3：回放上次录制的动作
    case 'M': goHome();       break;  // 按键4：回到初始姿态

    // ===== 按钮板按键 5~8 =====
    // 按键5：绘图模式切换 LINE / TRI / POLY / CURVE
    case 'F':
      // 切换模式不清空示教点：折线和曲线可以共用同一组点
      drawMode = (drawMode + 1) % MODE_COUNT;
      Serial.print(F("绘图模式: "));
      Serial.print(modeName(drawMode));
      // 顺手提示这个模式需要几个点，避免记错点数白画一次
      if (drawMode == MODE_LINE) {
        Serial.println(F("  （2 个点）"));
      } else if (drawMode == MODE_TRI) {
        Serial.println(F("  （正好 3 个点，自动闭合）"));
      } else if (drawMode == MODE_POLY) {
        Serial.println(F("  （5 个点，开放折线）"));
      } else {
        Serial.println(F("  （3~5 个点，平滑曲线）"));
      }
      break;

    // 按键6：开始 / 暂停 / 继续（同一个字符，按当前状态区分语义）
    case 'D': drawStart();        break;
    case 'I': teachRecordPoint(); break;  // 按键7：记录当前示教点
    // 按键8：取消绘制并回待机（没有在绘制时就是普通的回中）
    case 'Q':
      if (drawActing != 0) {
        drawCancel();
      } else {
        goHome();
      }
      break;

    case 'U':   // 串口备用：直接继续绘制（不用靠 D 键循环）
      if (drawActing == 4) {
        drawActing = 2;
        Serial.println(F("继续绘制"));
      } else {
        Serial.println(F("当前不在暂停状态"));
      }
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

      // 数字 1~6 是任务二的摇杆标定键，要在"坐标指令缓冲区"之前拦下来，
      // 否则会被当成坐标指令的字符塞进 serialLine
      if (name >= '1' && name <= '6') {
        armDataCmd(name);
      } else if (name == 'O' || name == 'S' || name == 'H' || name == 'L' || 
          name == 'P' || name == 'A' || name == 'B' || name == 'C' || 
          name == 'M' || name == 'N' || name == 'R' || name == 'T' ||
          name == 'K' ||
          name == 'F' || name == 'D' || name == 'I' || name == 'Q' ||
          name == 'U') {
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

// 读取按钮板通过软串口发来的单字符指令
// 按钮板一次只发一个字符，不带回车换行，所以收到就直接执行
void readBtnCmd() {
  while (btnSerial.available() > 0) {
    char data = btnSerial.read();

    // 忽略换行、回车等空白字符，避免按钮板误发时触发"无法识别指令"
    if (data == '\r' || data == '\n') continue;

    char name = data;
    if (name >= 'a' && name <= 'z') name = name - 'a' + 'A';

    Serial.print(F("[按钮板] "));
    Serial.println(name);

    armDataCmd(name);
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

  // 初始化与按钮板通信的软串口
  btnSerial.begin(9600);
  Serial.println(F("按钮板串口就绪: RX=D12 TX=D13"));

  // 机械臂上电先回到设定的初始角度
  servoWrite('b', bPos);
  servoWrite('f', fPos);
  servoWrite('r', rPos);
  servoWrite('c', cPos);
}

void loop() {
  readCmd();       // 电脑（硬件串口）
  readBtnCmd();    // 按钮板（软串口）
  stepJointTarget();
  homeCtrl();
  actionCtrl();
  playCtrl();
  drawCtrl();      // 任务五：绘制推进

  // 所有自动流程都空闲，才允许摇杆控制
  if (motionState == 0 && homeStep == 0 && actionStep == 0 &&
      playStage == 0 && drawActing == 0) {
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