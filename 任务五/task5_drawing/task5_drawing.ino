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
  | J | 五点模式首点I后，开/关模型锁高摇杆示教（串口） |
  | E | 五点模式空闲时清空示教点，保持原位，重新从第1点记录 |
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
  直线和三角形无需示教：把笔尖轻放纸面起点，D绘制固定尺寸图形。
  D 开始 / 暂停 / 继续，U 继续，Q 先垂直抬笔再回中。
  F 空闲时依次切换：直线 -> 三角形 -> 五点直线 -> 五点曲线 -> 直线。
  直线长30mm，沿当前底座朝向向外画；等边三角形边长30mm，末边经过起点后多画7.5mm。
  五点直线和五点曲线仍用I记录5点，按 1->2->3->4->5 连接，不闭合。
  五点模式保留各点X/Y，绘图Z统一取第1点；原始高度差超过5mm拒绝。
  端点用任务四正解换成 XYZ；在 XYZ 空间插值；每个采样点用逆解换成舵机角。
  开始前检查整条路径：垂直抬笔 -> 悬空平移 -> 落笔 -> 绘制 -> 垂直收笔。
  直线模式不是固定底座，也不是只转底座，而是三个关节按逆解同步更新。
  drawSpacingMm 控制采样距离，drawStepMs 控制最小节拍，DSD 限制角度变化速度。
  三角形第2/3顶点各等待至少300ms；末边仍延长25%，最终等待600ms再抬笔。
  五点首点I后可串口J锁高，摇杆只调XY；后臂/夹爪轴忽略，再J恢复。
  按 H/L 调整 DSD。整数角度输出有量化误差，实际精度取决于机构和标定。
  异常停止后 Q 尝试抬笔退出；若无法抬笔，M 解除锁定并保持原位，摇杆手动处理。
接线：
  底座舵机 D9，前臂舵机 D8，后臂舵机 D6，夹爪舵机 D7。
  底座摇杆 A0，前臂摇杆 A3，后臂摇杆 A1，夹爪摇杆 A2。

串口波特率：9600。
x,y,z 指令以回车或换行结束。
DSD 是每移动 1 度后的等待时间，DSD 越小，运行越快。

使用前应核对舵机转向、机械限位和夹爪开合角度。

实机验证事项：尺寸、舵机映射、笔尖偏移、纸面高度、抬笔间隙和运动速度。
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

// 毫米距离/角度比较的微小浮点容差；绘图另预留 0.5mm 奇异边界余量
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
  if (!isfinite(X) || !isfinite(Y) || !isfinite(Z)) return false;
  // R = sqrt(X^2 + Y^2)
  float R = sqrt(X * X + Y * Y);

  if (R < ARM_EPS) return false; // 回转轴上底座角不唯一

  // theta = atan2(Y, X)。注意是 atan2 不是 atan，能分四个象限
  theta = degrees(atan2(Y, X));

  // 扣掉固定偏移，化成两杆问题
  float r = R - L3a - L3b;
  float z = Z - h;

  // d = 目标点到肩轴的距离
  float d = sqrt(r * r + z * z);

  // 可达性判定：|L1-L2| <= d <= L1+L2
  // 故意不夹紧到边界，明显不可达就返回失败
  if (d < ARM_EPS) return false; // 防止肩轴附近除零
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

// ==================== 任务五：任务四公式驱动的笛卡尔轨迹 ====================
// 对照 MeArm 官方库（2026-10-15）：https://github.com/MeArm/MeArm-Arduino
// MeArm.cpp 的 moveToXYZ 使用XYZ插值；ik.cpp/fk.cpp的小臂角为绝对倾角。
// 此处保留任务四坐标与现有舵机标定，采用millis状态机，不引入官方阻塞delay循环。
// 示教保存舵机指令角；规划时先做正解，在 XYZ 空间插值，再逐点逆解。
// 由纸面实测姿态修正映射：小臂绝对倾角 gamma=r-90，不是相对角 beta=90-r。
// 文档定义 gamma=alpha+beta-90，因此 beta=f+r-180；逆映射 r=alpha+beta。
// 尺寸和零位仍需更多实测验证，下面的径向修正是单组测量的初步标定。
const int teachMax = 5;
JointPoint teachPoint[teachMax];
int teachCount = 0;
// J为可选锁高示教；关闭时完全沿用旧摇杆控制。缓存未取整目标，避免逐步FK造成高度漂移。
CartesianPoint teachPlanePoint[teachMax];
byte teachPlaneMask = 0;
bool planeTeachActive = false, planeTeachReady = false, planeTeachBlocked = false;
CartesianPoint planeTeachTarget;
JointPoint planeTeachJoint;
int planeTeachClaw = 0;
const float planeTeachStepMm = 0.5f;
unsigned long planeTeachLastStep = 0, planeTeachIntervalMs = 35;
// 模式顺序与 F 键一致；“五点直线”指相邻示教点之间逐段画直线。
const int MODE_LINE = 0; // 固定直线：当前纸面位置起笔，无需示教
const int MODE_TRI = 1; // 固定等边三角形：当前纸面位置为顶点，自动闭合
const int MODE_POLY = 2; // 五点直线：5点，1->2->3->4->5，不闭合
const int MODE_CURVE = 3; // 五点曲线：5点，平滑经过各点，不闭合
const int MODE_COUNT = 4;
int drawMode = MODE_LINE;
const float fixedLineLengthMm = 30.0f;
const float fixedTriangleSideMm = 30.0f;
const float triangleLastEdgeExtraRatio = 0.25f; // 最后一边经过起点后继续25%；0可关闭
// 三角形前三条边的顶点不变；额外延长最后一边，结束后停笔等待。
const unsigned long triangleCornerHoldMs = 300; // 第2/3顶点落笔等待，减小舵机滞后导致的圆角
const unsigned long triangleCloseHoldMs = 600; // 延长终点下发后保持落笔，再抬笔
JointPoint triangleStartJoint; // 起笔时实际下发的三舵机指令角，不是传感器反馈
// 首边沿起笔时底座朝向向外；三角形第三顶点位于该方向逆时针一侧。
// 同 f=113/r=10，底座114与80度，纸面弦长65mm；假设底座1:1且L1=L2=80mm。
// 推得固定水平偏移合计66.01mm，相对名义L3a+L3b=75mm修正-8.99mm。
// 这是任务五的有效径向修正，不代表单独测得L3a或L3b，也未修改任务四公式。
const float drawRadialCorrectionMm = -8.99f;
const float drawSpacingMm = 1.0f;
const float drawLiftMm = 10.0f; // 优先尝试的抬笔高度
const float drawMinLiftMm = 2.0f; // 不能达到此高度则拒绝，实际笔尖离纸间隙需实测
const float drawLiftStepMm = 1.0f; // 预检失败时逐级减小，运动前选定
float drawSelectedLiftMm = drawLiftMm;
const float drawSingularMarginMm = 0.5f; // 数值余量；另保留舵机限位及每点3度步长检查
// 五点模式锁定第1示教点的模型Z，保留各点X/Y；不是自动标定或接触检测。
const float drawMaxPlaneCorrectionMm = 5.0f; // 原始高度偏差超过此值拒绝，避免误记悬空点
const int drawMaxJointStep = 3;
const unsigned long drawStepMs = 35;

// 流式计算，不再保存 120 个关节路径点，也不会满缓冲后悄悄截断。
CartesianPoint drawPoint[teachMax];
int drawPointCount = 0; // 本次路径点数，与保存的teachCount独立，固定图形不覆盖示教
CartesianPoint approachPoint[4];
CartesianPoint liftFrom, liftTo;
// 0 空闲，1 抬笔/移到起点/落笔，2 绘图，4 暂停，5 完成抬笔，6 取消抬笔
int drawActing = 0;
int drawResumeState = 0;
int drawSegment = 0;
unsigned int drawSample = 0;
unsigned int drawSamples = 0;
unsigned long lastDrawStep = 0;
unsigned long drawIntervalMs = drawStepMs;
DrawError drawError = DRAW_OK;
unsigned int drawErrorSample = 0;
float drawLastDistance = 0.0f;
float drawLimitB = 0, drawLimitF = 0, drawLimitR = 0; // 失败时保留未取整的逆解角，用于诊断

CartesianPoint jointToCartesian(const JointPoint &p) {
  CartesianPoint out;
  forwardKinematics(p.b - 90.0f, 180.0f - p.f, p.f + p.r - 180.0f,
                    out.x, out.y, out.z);
  float thetaRad = radians(p.b - 90.0f);
  out.x += drawRadialCorrectionMm * cos(thetaRad);
  out.y += drawRadialCorrectionMm * sin(thetaRad);
  return out;
}

// 逆解后先检查浮点角度的限位，再四舍五入。禁止用限幅掩盖不可达点。
bool cartesianToJoint(const CartesianPoint &p, JointPoint &out) {
  drawError = DRAW_OK;
  float theta, alpha, beta;
  float radius = sqrt(p.x * p.x + p.y * p.y);
  float radial = radius - L3a - L3b - drawRadialCorrectionMm;
  float vertical = p.z - h;
  float d = sqrt(radial * radial + vertical * vertical);
  drawLastDistance = d;
  if (!isfinite(d)) { drawError = DRAW_NONFINITE; return false; }
  if (radius < ARM_EPS) { drawError = DRAW_AXIS; return false; }
  if (d > L1 + L2 + ARM_EPS || d < fabs(L1 - L2) - ARM_EPS) {
    drawError = DRAW_UNREACHABLE; return false;
  }
  if (d < fabs(L1 - L2) + drawSingularMarginMm ||
      d > L1 + L2 - drawSingularMarginMm) {
    drawError = DRAW_SINGULAR_MARGIN; return false;
  }
  // 从笔尖有效坐标还原到任务四的名义参考坐标，再调用原逆解。
  float modelRadius = radius - drawRadialCorrectionMm;
  if (!inverseKinematics(p.x * modelRadius / radius, p.y * modelRadius / radius,
                         p.z, theta, alpha, beta)) {
    drawError = DRAW_UNREACHABLE; return false;
  }
  float b = theta + 90.0f;
  float f = 180.0f - alpha;
  float r = alpha + beta; // 小臂绝对倾角gamma=r-90
  if (b < bMin - ARM_EPS || b > bMax + ARM_EPS ||
      f < fMin - ARM_EPS || f > fMax + ARM_EPS ||
      r < rMin - ARM_EPS || r > rMax + ARM_EPS) {
    drawLimitB = b; drawLimitF = f; drawLimitR = r;
    drawError = DRAW_JOINT_LIMIT; return false;
  }
  out.b = (int)round(b);
  out.f = (int)round(f);
  out.r = (int)round(r);
  return true;
}

void reportLimitAngle(char name, float angle, int low, int high) {
  if (angle >= low - ARM_EPS && angle <= high + ARM_EPS) return;
  Serial.print(name); Serial.print(F("要求角度=")); Serial.print(angle, 3);
  Serial.print(F("，允许[")); Serial.print(low); Serial.print(','); Serial.print(high);
  Serial.println(F("]度"));
}

void reportDrawError() {
  switch (drawError) {
    case DRAW_NONFINITE: Serial.println(F("坐标非有限数，请检查参数")); break;
    case DRAW_AXIS: Serial.println(F("目标在回转轴附近，底座角不唯一")); break;
    case DRAW_UNREACHABLE:
      Serial.print(F("超出当前尺寸模型的可达距离，d(mm)="));
      Serial.println(drawLastDistance, 2); break;
    case DRAW_SINGULAR_MARGIN:
      Serial.print(F("进入奇异边界余量，d(mm)=")); Serial.print(drawLastDistance, 2);
      Serial.print(F("，余量(mm)=")); Serial.println(drawSingularMarginMm, 2); break;
    case DRAW_JOINT_LIMIT:
      Serial.println(F("逆解舵机角超出配置限位"));
      reportLimitAngle('b', drawLimitB, bMin, bMax);
      reportLimitAngle('f', drawLimitF, fMin, fMax);
      reportLimitAngle('r', drawLimitR, rMin, rMax);
      break;
    case DRAW_STEP_LIMIT: Serial.println(F("相邻采样角度变化超过3度")); break;
    case DRAW_SAMPLE_LIMIT: Serial.println(F("采样数超限或路径数据无效")); break;
    default: Serial.println(F("路径检查未通过")); break;
  }
}
int jointDifference(const JointPoint &a, const JointPoint &b) {
  int result = abs(a.b - b.b);
  if (abs(a.f - b.f) > result) result = abs(a.f - b.f);
  if (abs(a.r - b.r) > result) result = abs(a.r - b.r);
  return result;
}

float pointDistance(const CartesianPoint &a, const CartesianPoint &b) {
  float x = b.x - a.x, y = b.y - a.y, z = b.z - a.z;
  return sqrt(x * x + y * y + z * z);
}

float catmullRom(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t, t3 = t2 * t;
  return 0.5f * (2 * p1 + (-p0 + p2) * t +
      (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
      (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
}

int drawingSegmentCount() {
  return drawMode == MODE_TRI ? drawPointCount : drawPointCount - 1;
}

void segmentEnds(int phase, int segment, CartesianPoint &a, CartesianPoint &b) {
  if (phase == 1) {
    a = approachPoint[segment]; b = approachPoint[segment + 1];
  } else if (phase == 2) {
    if (drawMode == MODE_TRI && segment == 2) {
      a = drawPoint[2]; b = drawPoint[0]; // 先精确经过原起点
    } else if (drawMode == MODE_TRI && segment == 3) {
      a = drawPoint[0]; b = drawPoint[3]; // 沿最后一边方向继续延长
    } else {
      a = drawPoint[segment]; b = drawPoint[(segment + 1) % drawPointCount];
    }
  } else {
    a = liftFrom; b = liftTo;
  }
}

CartesianPoint segmentPoint(int phase, int segment, float t) {
  CartesianPoint a, b, out;
  segmentEnds(phase, segment, a, b);
  if (phase == 2 && drawMode == MODE_CURVE) {
    int i0 = segment == 0 ? 0 : segment - 1;
    int i3 = segment + 2 < drawPointCount ? segment + 2 : drawPointCount - 1;
    out.x = catmullRom(drawPoint[i0].x, a.x, b.x, drawPoint[i3].x, t);
    out.y = catmullRom(drawPoint[i0].y, a.y, b.y, drawPoint[i3].y, t);
    out.z = catmullRom(drawPoint[i0].z, a.z, b.z, drawPoint[i3].z, t);
  } else {
    // 端点直接返回，避免浮点加减让闭合坐标出现末位偏差。
    if (t >= 1.0f) return b;
    // 任务五直线的关键：插值的是坐标，不是舵机角度。
    out.x = a.x + (b.x - a.x) * t;
    out.y = a.y + (b.y - a.y) * t;
    out.z = a.z + (b.z - a.z) * t;
  }
  return out;
}

unsigned int segmentSamples(int phase, int segment) {
  CartesianPoint a, b;
  segmentEnds(phase, segment, a, b);
  float lengthBound = pointDistance(a, b);
  if (phase == 2 && drawMode == MODE_CURVE) {
    int i0 = segment == 0 ? 0 : segment - 1;
    int i3 = segment + 2 < drawPointCount ? segment + 2 : drawPointCount - 1;
    // 用导数上界取样，曲线也在 XYZ 空间计算，并检查中间点是否越界。
    lengthBound = 4.0f * (pointDistance(drawPoint[i0], a) + lengthBound +
                         pointDistance(b, drawPoint[i3]));
  }
  if (!isfinite(lengthBound) || lengthBound > 4096.0f * drawSpacingMm) return 0;
  unsigned int count = (unsigned int)ceil(lengthBound / drawSpacingMm);
  return count > 0 ? count : 1;
}

bool validateSegment(int phase, int segment, JointPoint &previous) {
  drawErrorSample = 0;
  unsigned int count = segmentSamples(phase, segment);
  if (count == 0) { drawError = DRAW_SAMPLE_LIMIT; return false; }
  for (unsigned int k = 0; k <= count; k++) {
    drawErrorSample = k;
    JointPoint next;
    if (!cartesianToJoint(segmentPoint(phase, segment, (float)k / count), next)) return false;
    if (jointDifference(previous, next) > drawMaxJointStep) {
      drawError = DRAW_STEP_LIMIT; return false;
    }
    previous = next;
  }
  return true;
}
const char* modeName(int m) {
  switch (m) {
    case MODE_LINE: return "直线";
    case MODE_TRI: return "三角形";
    case MODE_POLY: return "五点直线";
    default: return "五点曲线";
  }
}

// 记录与执行共用同一投影检查；只计算，不改示教数组、不输出舵机指令。
// raw是原始指令角FK；projected可携带锁高示教的未取整XY缓存。
bool validateTeachProjection(const CartesianPoint &raw, CartesianPoint &projected,
                             float paperZ, int index) {
  float dz = raw.z - paperZ;
  if (fabs(dz) > drawMaxPlaneCorrectionMm) {
    Serial.print(F("示教点 ")); Serial.print(index + 1);
    Serial.print(F(" 原始高度差(mm)=")); Serial.print(dz, 2);
    Serial.print(F("，超过允许修正量(mm)=")); Serial.println(drawMaxPlaneCorrectionMm, 1);
    return false;
  }
  projected.z = paperZ;
  JointPoint solved;
  if (!cartesianToJoint(projected, solved)) {
    Serial.print(F("示教点 ")); Serial.print(index + 1); Serial.print(F(" 统一高度后不可达："));
    reportDrawError();
    Serial.print(F("原始Z=")); Serial.print(raw.z, 2);
    Serial.print(F("，首点Z=")); Serial.println(paperZ, 2);
    return false;
  }
  return true;
}

void teachRecordPoint() {
  if (drawActing != 0 || motionState != 0 || homeStep != 0 ||
      actionStep != 0 || playStage != 0 || recording) {
    Serial.println(F("请在空闲时记录示教点"));
    return;
  }
  if (drawMode == MODE_LINE || drawMode == MODE_TRI) {
    Serial.println(F("固定图形无需示教；先切换五点模式再按I记录")); return;
  }
  if (planeTeachActive && (!planeTeachReady || joyDir(bJoy) || joyDir(fJoy))) {
    Serial.println(F("请先松开平面摇杆再记录")); return;
  }
  if (planeTeachActive && teachCount >= teachMax) {
    Serial.println(F("5点已满，D绘图；J退出锁高后可重新示教")); return;
  }
  // 先验证再提交：失败不占点号、不清旧组、不改缓存，也不移动机械臂。
  int index = teachCount >= teachMax ? 0 : teachCount;
  JointPoint p = {bPos, fPos, rPos}, solved;
  CartesianPoint xyz = jointToCartesian(p);
  float paperZ = index == 0 ? xyz.z : jointToCartesian(teachPoint[0]).z;
  Serial.print(F("候选示教点 ")); Serial.print(index + 1);
  Serial.print(F(" b=")); Serial.print(p.b);
  Serial.print(F(" f=")); Serial.print(p.f);
  Serial.print(F(" r=")); Serial.print(p.r);
  Serial.print(F(" 模型X=")); Serial.print(xyz.x, 2);
  Serial.print(F(" Y=")); Serial.print(xyz.y, 2);
  Serial.print(F(" 原始Z=")); Serial.println(xyz.z, 2);
  Serial.print(F("五点绘图锁定Z=")); Serial.print(paperZ, 2);
  Serial.print(F("，当前原始高度差(mm)=")); Serial.println(xyz.z - paperZ, 2);
  if (!cartesianToJoint(xyz, solved)) {
    Serial.println(F("未记录：原始姿态不可用")); reportDrawError(); return;
  }
  if (jointDifference(p, solved) > 1) {
    Serial.println(F("未记录：原始姿态与逆解分支不一致")); return;
  }
  CartesianPoint projected = planeTeachActive ? planeTeachTarget : xyz;
  if (!validateTeachProjection(xyz, projected, paperZ, index)) {
    Serial.println(F("未记录，点号不变；调整当前位置后再I")); return;
  }
  if (index == 0) teachPlaneMask = 0;
  teachPlaneMask &= ~(1 << index);
  if (planeTeachActive) {
    teachPlanePoint[index] = planeTeachTarget;
    teachPlaneMask |= 1 << index;
    Serial.print(F("已保存锁高目标 X=")); Serial.print(planeTeachTarget.x, 2);
    Serial.print(F(" Y=")); Serial.print(planeTeachTarget.y, 2);
    Serial.print(F(" Z=")); Serial.println(planeTeachTarget.z, 2);
  }
  teachPoint[index] = p;
  teachCount = index + 1;
  Serial.print(F("已记录示教点 ")); Serial.println(teachCount);
  if (index == 0) Serial.println(F("建议现在发送J启用锁高摇杆；未发送J仍为独立关节控制"));

}

void beginDrawPhase(int phase) {
  drawActing = phase;
  drawSegment = 0;
  drawSample = 0;
  drawSamples = segmentSamples(phase, 0);
  lastDrawStep = millis();
  drawIntervalMs = drawStepMs;
}

void resumeDrawing() {
  if (drawActing != 4) return;
  if (drawResumeState == 0) { Serial.println(F("异常停止不可继续，请按 Q 或 M")); return; }
  drawActing = drawResumeState;
  lastDrawStep = millis();
  Serial.println(F("继续绘制"));
}

void drawStart() {
  planeTeachActive = false; // 轨迹执行接管关节，不与锁高摇杆并行
  if (drawActing == 4) { resumeDrawing(); return; }
  if (drawActing == 1 || drawActing == 2 || drawActing == 5) {
    drawResumeState = drawActing;
    drawActing = 4;
    Serial.println(F("已暂停，D/U 继续，Q 取消"));
    return;
  }
  if (drawActing != 0 || motionState != 0 || homeStep != 0 ||
      actionStep != 0 || playStage != 0 || recording) {
    Serial.println(F("正在执行其他流程，请等待")); return;
  }
  bool fixedShape = drawMode == MODE_LINE || drawMode == MODE_TRI;
  if (!fixedShape && teachCount != 5) {
    Serial.println(F("五点直线和五点曲线需要正好5个示教点")); return;
  }
  JointPoint current = {bPos, fPos, rPos}, solved;
  if (fixedShape) {
    // 当前姿态作为本次固定图形起点与高度；不会自动探测纸面。
    drawPointCount = drawMode == MODE_LINE ? 2 : 3;
    drawPoint[0] = jointToCartesian(current);
    float angle = radians(bPos - 90.0f);
    float ux = cos(angle), uy = sin(angle);
    float size = drawMode == MODE_LINE ? fixedLineLengthMm : fixedTriangleSideMm;
    if (!isfinite(size) || size <= 0.0f) {
      Serial.println(F("固定图形尺寸必须为正数")); return;
    }
    drawPoint[1] = drawPoint[0];
    drawPoint[1].x += size * ux;
    drawPoint[1].y += size * uy;
    if (drawMode == MODE_TRI) {
      drawPoint[2] = drawPoint[0];
      float height = size * 0.8660254f;
      drawPoint[2].x += size * 0.5f * ux - height * uy;
      drawPoint[2].y += size * 0.5f * uy + height * ux;
      if (!isfinite(triangleLastEdgeExtraRatio) || triangleLastEdgeExtraRatio < 0.0f) {
        Serial.println(F("三角形延长比例必须为非负数")); return;
      }
      if (triangleLastEdgeExtraRatio > 0.0f) {
        drawPointCount = 4;
        drawPoint[3] = drawPoint[0];
        drawPoint[3].x += (drawPoint[0].x - drawPoint[2].x) * triangleLastEdgeExtraRatio;
        drawPoint[3].y += (drawPoint[0].y - drawPoint[2].y) * triangleLastEdgeExtraRatio;
        Serial.print(F("最后一边经过起点后延长(mm)="));
        Serial.println(size * triangleLastEdgeExtraRatio, 2);
      }
    }
    Serial.print(F("固定图形，以当前位置为纸面起点，尺寸(mm)=")); Serial.println(size, 1);
  } else {
    drawPointCount = teachCount;
    for (int i = 0; i < drawPointCount; i++) drawPoint[i] = jointToCartesian(teachPoint[i]);
  }
  for (int i = 0; i < drawPointCount; i++) {
    if (!cartesianToJoint(drawPoint[i], solved)) {
      Serial.print(F("路径顶点 ")); Serial.print(i + 1); Serial.print(F(" 检查失败："));
      reportDrawError(); return;
    }
    if ((!fixedShape && jointDifference(solved, teachPoint[i]) > 1) ||
        (fixedShape && i == 0 && jointDifference(solved, current) > 1)) {
      Serial.println(F("起点/示教姿态与逆解不一致，请检查分支或映射")); return;
    }

  }
  if (!fixedShape) {
    // 原始姿态的逆解分支已在上面检查。只投影本次轨迹，不覆盖teachPoint。
    float paperZ = drawPoint[0].z;
    for (int i = 0; i < drawPointCount; i++) {
      CartesianPoint raw = drawPoint[i];
      // 锁高点使用未取整的缓存XY；普通示教保留FK所得XY。
      if (teachPlaneMask & (1 << i)) drawPoint[i] = teachPlanePoint[i];
      if (!validateTeachProjection(raw, drawPoint[i], paperZ, i)) return;
    }
    Serial.print(F("五点路径统一纸面Z=")); Serial.println(paperZ, 2);
  }
  // 先检查纸面绘图本身，避免把画线不可达与抬笔高度不足混为一谈。
  JointPoint previous;
  if (!cartesianToJoint(drawPoint[0], previous)) { reportDrawError(); return; }
  for (int s = 0; s < drawingSegmentCount(); s++) {
    if (!validateSegment(2, s, previous)) {
      Serial.print(F("绘图段 ")); Serial.print(s + 1);
      Serial.print(F("，采样 ")); Serial.print(drawErrorSample); Serial.print(F("："));
      reportDrawError(); return;
    }
  }
  approachPoint[0] = jointToCartesian(current);
  float highestZ = approachPoint[0].z;
  for (int i = 0; i < drawPointCount; i++)
    if (drawPoint[i].z > highestZ) highestZ = drawPoint[i].z;
  const int endIndex = drawMode == MODE_TRI && drawPointCount == 3 ? 0 : drawPointCount - 1;
  liftFrom = drawPoint[endIndex];
  bool valid = false;
  // 固定10mm可能超出工作空间。逐级检查到2mm，只有整条接近/收笔路径通过才采用。
  for (float lift = drawLiftMm; lift >= drawMinLiftMm; lift -= drawLiftStepMm) {
    approachPoint[1] = approachPoint[0]; approachPoint[1].z = highestZ + lift;
    approachPoint[2] = drawPoint[0]; approachPoint[2].z = highestZ + lift;
    approachPoint[3] = drawPoint[0];
    liftTo = liftFrom; liftTo.z += lift;
    previous = current;
    valid = true;
    for (int s = 0; s < 3 && valid; s++) valid = validateSegment(1, s, previous);
    if (valid) {
      if (!cartesianToJoint(drawPoint[endIndex], previous)) { valid = false; continue; }
      valid = validateSegment(5, 0, previous);
    }
    if (valid) { drawSelectedLiftMm = lift; break; }
  }
  if (!valid) {
    Serial.print(F("绘图段可达，但接近/收笔无法满足最小抬笔高度(mm)="));
    Serial.println(drawMinLiftMm, 1); reportDrawError(); return;
  }
  Serial.print(F("路径检查通过，选定抬笔高度(mm)=")); Serial.println(drawSelectedLiftMm, 1);
  beginDrawPhase(1);
}

void drawCancel() {
  if (drawActing == 0 || drawActing == 6) return;
  JointPoint current = {bPos, fPos, rPos};
  liftFrom = jointToCartesian(current);
  bool valid = false;
  for (float lift = drawLiftMm; lift >= drawMinLiftMm; lift -= drawLiftStepMm) {
    liftTo = liftFrom; liftTo.z += lift;
    JointPoint previous = current;
    if (validateSegment(6, 0, previous)) {
      drawSelectedLiftMm = lift; valid = true; break;
    }
  }
  if (!valid) {
    drawResumeState = 0;
    drawActing = 4;
    Serial.println(F("已停止；取消抬笔路径不可达，保持位置，M 退出后手动处理"));
    reportDrawError(); return;
  }
  Serial.print(F("取消绘制，抬笔高度(mm)=")); Serial.println(drawSelectedLiftMm, 1);
  beginDrawPhase(6);
}

// 每次只发一个 XYZ 采样点对应的三个角度，不借用逐轴每次一度的通用运动器。
// 整数角度有量化误差，且无位置反馈；这不是舵机已实际到位的证明。
void drawCtrl() {
  if (drawActing == 0 || drawActing == 4) return;
  if (millis() - lastDrawStep < drawIntervalMs) return;
  if (drawSample >= drawSamples) {
    int segmentCount = drawActing == 1 ? 3 :
        (drawActing == 2 ? drawingSegmentCount() : 1);
    if (++drawSegment < segmentCount) {
      drawSample = 0;
      drawSamples = segmentSamples(drawActing, drawSegment);
    } else if (drawActing == 1) {
      if (drawMode == MODE_TRI) {
        triangleStartJoint.b = bPos;
        triangleStartJoint.f = fPos;
        triangleStartJoint.r = rPos;
        Serial.print(F("三角形起笔角 b=")); Serial.print(bPos);
        Serial.print(F(" f=")); Serial.print(fPos); Serial.print(F(" r=")); Serial.println(rPos);
      }
      Serial.println(F("开始按 XYZ 路径绘制")); beginDrawPhase(2); return;
    } else if (drawActing == 2) {
      beginDrawPhase(5); return;
    } else {
      bool canceled = drawActing == 6;
      drawActing = 0;
      Serial.println(canceled ? F("已抬笔，开始回中") : F("绘制完成，已抬笔"));
      if (canceled) goHome();
      return;
    }
  }
  JointPoint next, current = {bPos, fPos, rPos};
  CartesianPoint point = segmentPoint(drawActing, drawSegment,
                                     (float)(drawSample + 1) / drawSamples);
  bool triangleAtStart = drawMode == MODE_TRI && drawActing == 2 && drawSegment == 2;
  bool triangleAtEnd = drawMode == MODE_TRI && drawActing == 2 &&
                       drawSegment == drawingSegmentCount() - 1;
  bool segmentEndpoint = drawSample + 1 == drawSamples;
  bool solved = cartesianToJoint(point, next);
  if (solved && triangleAtStart && segmentEndpoint) {
    // 同一顶点还原同一组指令；仍通过下面的每步角度检查，禁止突跳补线。
    next = triangleStartJoint;
  }
  if (solved && jointDifference(current, next) > drawMaxJointStep) {
    drawError = DRAW_STEP_LIMIT; solved = false;
  }
  if (!solved) {
    drawResumeState = 0;
    drawActing = 4;
    Serial.println(F("轨迹异常，已停止；Q 尝试抬笔取消，M 退出后手动处理")); reportDrawError(); return;
  }
  int difference = jointDifference(current, next);
  // 在同一轮循环更新全部关节，避免底座沿圆弧运动时大小臂未补偿。
  servoWrite('b', next.b); servoWrite('f', next.f); servoWrite('r', next.r);
  targetB = bPos; targetF = fPos; targetR = rPos; targetC = cPos;
  drawSample++;
  lastDrawStep = millis();
  drawIntervalMs = (unsigned long)difference * (unsigned long)DSD;
  if (drawIntervalMs < drawStepMs) drawIntervalMs = drawStepMs;
  if (drawMode == MODE_TRI && drawActing == 2 && drawSegment < 2 && segmentEndpoint) {
    // 先下发精确顶点，保持落笔等待，再转向下一边；millis计时，D/Q仍可响应。
    if (drawIntervalMs < triangleCornerHoldMs) drawIntervalMs = triangleCornerHoldMs;
  }
  if (triangleAtStart && segmentEndpoint) {
    Serial.print(F("经过起点目标角 b=")); Serial.print(bPos);
    Serial.print(F(" f=")); Serial.print(fPos); Serial.print(F(" r=")); Serial.println(rPos);
  }
  if (triangleAtEnd && segmentEndpoint) {
    // 延长段也完成后才等待并收笔，各边速度保持原规则。
    if (drawIntervalMs < triangleCloseHoldMs) drawIntervalMs = triangleCloseHoldMs;
    Serial.print(F("三角形最终目标角 b=")); Serial.print(bPos);
    Serial.print(F(" f=")); Serial.print(fPos); Serial.print(F(" r=")); Serial.println(rPos);
    Serial.print(F("保持落笔等待(ms)=")); Serial.println(drawIntervalMs);
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

// 任务五可选平面摇杆：第1点贴纸并I后发送J；原底座轴控制Y，原前臂轴控制X。
// 只锁定数学模型Z，无接触反馈。后臂/夹爪摇杆在锁高期间忽略，不自动下压笔尖。
void planeTeachToggle() {
  if (planeTeachActive) {
    planeTeachActive = false;
    Serial.println(F("已退出锁高示教，恢复关节摇杆")); return;
  }
  if ((drawMode != MODE_POLY && drawMode != MODE_CURVE) || teachCount != 1 ||
      drawActing || motionState || homeStep || actionStep || playStage || recording) {
    Serial.println(F("锁高要求：五点模式、空闲、刚记录第1点")); return;
  }
  JointPoint current = {bPos, fPos, rPos}, solved;
  if (jointDifference(current, teachPoint[0]) != 0) {
    Serial.println(F("已离开第1点，请重新示教后再J")); return;
  }
  CartesianPoint target = jointToCartesian(current);
  if (!cartesianToJoint(target, solved) || jointDifference(current, solved) > 1) {
    Serial.println(F("首点无法锁高，请检查姿态/限位")); return;
  }
  planeTeachTarget = target; planeTeachJoint = current; planeTeachClaw = cPos;
  planeTeachReady = false; planeTeachBlocked = false; planeTeachActive = true;
  planeTeachLastStep = millis(); planeTeachIntervalMs = drawStepMs;
  Serial.println(F("锁高示教已开：摇杆先回中；底座轴=Y，前臂轴=X，I记录，J退出"));
}

void planeTeachCtrl() {
  if (!planeTeachActive) return;
  JointPoint current = {bPos, fPos, rPos};
  if ((drawMode != MODE_POLY && drawMode != MODE_CURVE) || recording ||
      drawActing || motionState || homeStep || actionStep || playStage ||
      cPos != planeTeachClaw || jointDifference(current, planeTeachJoint) != 0) {
    planeTeachActive = false;
    Serial.println(F("姿态或流程改变，锁高已退出")); return;
  }
  int dx = -joyDir(fJoy), dy = -joyDir(bJoy);
  if (!planeTeachReady) {
    // 切换时必须所有摇杆回中，防止旧推杆动作立即变成平面运动。
    if (!dx && !dy && !joyDir(rJoy) && !joyDir(cJoy)) planeTeachReady = true;
    planeTeachLastStep = millis(); return;
  }
  if (millis() - planeTeachLastStep < planeTeachIntervalMs) return;
  if (!dx && !dy) { planeTeachBlocked = false; return; }
  CartesianPoint candidate = planeTeachTarget;
  float step = planeTeachStepMm * ((dx && dy) ? 0.70710678f : 1.0f);
  candidate.x += dx * step; candidate.y += dy * step;
  JointPoint next;
  bool valid = cartesianToJoint(candidate, next);
  if (valid && jointDifference(current, next) > drawMaxJointStep) {
    drawError = DRAW_STEP_LIMIT; valid = false;
  }
  planeTeachLastStep = millis();
  if (!valid) {
    // 拒绝该步且不积累越界目标；反向推杆仍可离开边界，日志只报一次避免堵串口。
    if (!planeTeachBlocked) { Serial.print(F("锁高移动受限：")); reportDrawError(); }
    planeTeachBlocked = true; return;
  }
  planeTeachBlocked = false;
  planeTeachIntervalMs = (unsigned long)jointDifference(current, next) * (unsigned long)DSD;
  if (planeTeachIntervalMs < drawStepMs) planeTeachIntervalMs = drawStepMs;
  servoWrite('b', next.b); servoWrite('f', next.f); servoWrite('r', next.r);
  targetB = bPos; targetF = fPos; targetR = rPos; targetC = cPos;
  planeTeachTarget = candidate; planeTeachJoint = next;
}

// 处理上位机固定OSHL等指令
void armDataCmd(char name) {
  // 允许上位机发送小写字母
  if (name >= 'a' && name <= 'z') name = name - 'a' + 'A';
  // 绘图独占关节；暂停、继续、取消必须先于通用的忙碌判断。
  if (drawActing != 0) {
    if (name == 'M' && drawActing == 4 && drawResumeState == 0) {
      drawActing = 0;
      Serial.println(F("已退出异常锁定，保持原位，请用摇杆手动抬笔"));
      return;
    }
    if (name != 'D' && name != 'Q' && name != 'U' &&
        name != 'P' && name != 'H' && name != 'L') {
      Serial.println(F("绘图期间仅接受 D/Q/U/P/H/L")); return;
    }
  }
  if ((motionState != 0 || homeStep != 0 || actionStep != 0 || playStage != 0) &&
      name != 'P' &&
      name != 'H' &&
      name != 'L') {
    Serial.println(F("正在自动运动，暂不接受此指令"));
    return;
  }
  // 只有记录/查询/调速/J允许保持锁高；其他指令先释放关节控制权。
  if (planeTeachActive && name != 'I' && name != 'P' && name != 'H' &&
      name != 'L' && name != 'J') planeTeachActive = false;
  switch (name) {
    case 'J': planeTeachToggle(); break;
    case 'E': // 只清任务五数据，绝不移动或清除任务二/三标定及录制
      if ((drawMode != MODE_POLY && drawMode != MODE_CURVE) || recording) {
        Serial.println(F("仅五点模式空闲时可清空示教")); break;
      }
      teachCount = 0; teachPlaneMask = 0;
      Serial.println(F("示教点已清空，保持原位；I记录首点后可J锁高")); break;
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
    // 按键5：依次切换 直线 / 三角形 / 五点直线 / 五点曲线
    case 'F':
      // 切换模式不清空示教点：五点直线和五点曲线可以共用同一组5点
      drawMode = (drawMode + 1) % MODE_COUNT;
      Serial.print(F("绘图模式: "));
      Serial.print(modeName(drawMode));
      // 顺手提示这个模式需要几个点，避免记错点数白画一次
      if (drawMode == MODE_LINE) {
        Serial.println(F("  （固定直线，无需示教；纸面起点按D）"));
      } else if (drawMode == MODE_TRI) {
        Serial.println(F("  （固定等边三角形，无需示教；纸面顶点按D）"));
      } else if (drawMode == MODE_POLY) {
        Serial.println(F("  （5点，首点定高度，逐段直线，不闭合）"));
      } else {
        Serial.println(F("  （5点，首点定高度，平滑曲线，不闭合）"));
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
        resumeDrawing();
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
  if (drawActing != 0 || motionState != 0 || playStage != 0 || homeStep != 0 || actionStep != 0) {
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
    planeTeachActive = false;
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
      if (lineLength == 0 && name >= '1' && name <= '6') {
        armDataCmd(name);
      } else if (lineLength == 0 && (name == 'O' || name == 'S' || name == 'H' || name == 'L' ||
          name == 'P' || name == 'A' || name == 'B' || name == 'C' || 
          name == 'M' || name == 'N' || name == 'R' || name == 'T' ||
          name == 'K' || name == 'J' || name == 'E' ||
          name == 'F' || name == 'D' || name == 'I' || name == 'Q' ||
          name == 'U')) {
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
      if (planeTeachActive) planeTeachCtrl();
      else joyCtrl();
    }
  } else {
    lastJoyStep = millis();
  }
  recordCtrl();
}