/*
  按键控制板程序 · Arduino Uno R3（8 键，无屏幕）
  ==================================================================
  这块板只做三件事，机械臂的动作全部由机械臂端完成：
    1. 扫描 8 个按键，做软件去抖
    2. 按下瞬间通过软串口发 1 个字符给机械臂（长按不重复发，松开不发）
    3. 每发一次，就往 USB 串口（115200）打一行，便于观察和验证

  按键分配（一键一符，不用短按长按）
    任务三  D4='N' 循环取放    D5='R' 录制开始/停止
            D6='T' 回放        D7='M' 回中
    任务五  D8='F' 模式切换    D9='D' 开始/暂停/继续
            D10='I' 记点       D11='Q' 取消并回中
            模式与点数（机械臂端决定）：直线 2 点 / 三角形正好 3 点
                        / 五点折线 5 点 / 平滑曲线 3~5 点

  接线（和机械臂只连两根）
    本板 D3(TX)  → 机械臂 D12(RX)
    本板 GND     ↔ 机械臂 GND
    本板 D0/D1 空着（USB 下载口），所以烧录不用拔线

  引脚占用
    D2/D3   软串口（本程序只发送，D2 可以不接）
    D4-D11  八个按键（一端接引脚，一端接 GND，内部上拉）
    D0/D1   硬件串口，接电脑看输出（115200）
    D12/D13、A0-A5 空着

  为什么用软串口而不是 D0/D1：D0/D1 就是 USB 口，占了就没法一边烧录/看打印
  一边和机械臂通信。分开之后两块板烧录都不用拔线。

  开发板：Arduino Uno        需要的库：无
*/

#include <SoftwareSerial.h>

// ==================== 引脚 ====================
const uint8_t PIN_BTN1 = 4;    // 任务三 循环取放
const uint8_t PIN_BTN2 = 5;    // 任务三 录制
const uint8_t PIN_BTN3 = 6;    // 任务三 回放
const uint8_t PIN_BTN4 = 7;    // 任务三 回中
const uint8_t PIN_BTN5 = 8;    // 任务五 模式
const uint8_t PIN_BTN6 = 9;    // 任务五 开始/暂停/继续
const uint8_t PIN_BTN7 = 10;   // 任务五 记点
const uint8_t PIN_BTN8 = 11;   // 任务五 取消并回中

// 与机械臂通信的软串口：RX, TX。只发送，RX 可以不接
const uint8_t PIN_ARM_RX = 2;
const uint8_t PIN_ARM_TX = 3;
SoftwareSerial armSerial(PIN_ARM_RX, PIN_ARM_TX);

// ==================== 参数 ====================
const long ARM_BAUD = 9600;    // 与机械臂端 btnSerial.begin(9600) 一致
const long DBG_BAUD = 115200;  // USB 串口，给电脑看
const unsigned long DEBOUNCE_MS = 30;    // 机械按键抖动 5~20ms，取 30ms
const unsigned long SEND_GAP_MS = 20;    // 发完停一下，避免机械臂还没读完就连发

// ==================== 按键表 ====================
// 表驱动：加按键只要在数组里加一行，再到 nameOfKey() 加一个 case
struct ButtonKey {
  uint8_t pin;               // 引脚号
  char    code;              // 按下时发给机械臂的字符
  uint8_t lastRaw;           // 上次读到的原始电平
  uint8_t stable;            // 去抖后已确认的电平
  unsigned long lastChange;  // 原始电平上次变化的时刻
};

ButtonKey keys[] = {
  { PIN_BTN1, 'N', HIGH, HIGH, 0 },  // 循环取放
  { PIN_BTN2, 'R', HIGH, HIGH, 0 },  // 录制开始/停止
  { PIN_BTN3, 'T', HIGH, HIGH, 0 },  // 回放
  { PIN_BTN4, 'M', HIGH, HIGH, 0 },  // 回中
  { PIN_BTN5, 'F', HIGH, HIGH, 0 },  // 模式切换
  { PIN_BTN6, 'D', HIGH, HIGH, 0 },  // 开始/暂停/继续
  { PIN_BTN7, 'I', HIGH, HIGH, 0 },  // 记点
  { PIN_BTN8, 'Q', HIGH, HIGH, 0 },  // 取消并回中
};

const uint8_t KEY_COUNT = sizeof(keys) / sizeof(keys[0]);

// 累计按键次数：用来确认"一次按下只触发一次"
unsigned int pressCount = 0;

// ==================== 中文功能名（只用于串口打印，放 Flash 不占 RAM） ====================
const char N_LOOP[]   PROGMEM = "循环取放";
const char N_REC[]    PROGMEM = "录制";
const char N_PLAY[]   PROGMEM = "回放";
const char N_HOME[]   PROGMEM = "回中";
const char N_MODE[]   PROGMEM = "模式";
const char N_DRAW[]   PROGMEM = "开始/暂停/继续";
const char N_TEACH[]  PROGMEM = "记点";
const char N_CANCEL[] PROGMEM = "取消/回中";

PGM_P nameOfKey(uint8_t idx) {
  switch (idx) {
    case 0:  return N_LOOP;
    case 1:  return N_REC;
    case 2:  return N_PLAY;
    case 3:  return N_HOME;
    case 4:  return N_MODE;
    case 5:  return N_DRAW;
    case 6:  return N_TEACH;
    default: return N_CANCEL;
  }
}

// ==================== 初始化 ====================
void setup() {
  Serial.begin(DBG_BAUD);
  armSerial.begin(ARM_BAUD);

  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    pinMode(keys[i].pin, INPUT_PULLUP);      // 内部上拉，不需要外接电阻
    keys[i].lastRaw = digitalRead(keys[i].pin);
    keys[i].stable = keys[i].lastRaw;
  }

  Serial.println(F("按键控制板就绪（Uno，8 键，无屏幕）"));
  Serial.println(F("任务三: 1=N 循环取放   2=R 录制   3=T 回放   4=M 回中"));
  Serial.println(F("任务五: 5=F 模式   6=D 开始/暂停/继续   7=I 记点   8=Q 取消/回中"));
  Serial.println(F("每按一次在下面打一行，行末数字是累计次数（一次按下应该只 +1）"));
  Serial.println(F("----"));
}

// ==================== 按键扫描 ====================
// 返回 true 表示刚刚按下（HIGH → LOW 的瞬间，只返回一次）
//
// 计时确认法：原始电平一变先记时刻、但不改"已确认状态"，
// 只有新电平稳定保持超过 DEBOUNCE_MS 才认可。
// 抖动期间来回跳变只会不断重置计时器，永远不会被当成有效按下。
bool keyJustPressed(ButtonKey &k) {
  uint8_t raw = digitalRead(k.pin);
  unsigned long now = millis();

  if (raw != k.lastRaw) {
    k.lastRaw = raw;
    k.lastChange = now;
    return false;
  }

  if (raw != k.stable && (now - k.lastChange) >= DEBOUNCE_MS) {
    k.stable = raw;
    // 只有按下（LOW）才触发；松开不触发，所以长按不会重复发
    return (k.stable == LOW);
  }

  return false;
}

// ==================== 主循环 ====================
void loop() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    if (keyJustPressed(keys[i])) {
      // 先发字符：软串口发送会短暂关中断，和打印分开做
      armSerial.write(keys[i].code);

      pressCount++;
      Serial.print(F("["));
      Serial.print(pressCount);
      Serial.print(F("] 按键"));
      Serial.print(i + 1);
      Serial.print(F(" → 发送 "));
      Serial.print(keys[i].code);
      Serial.print(F("  "));
      Serial.println((const __FlashStringHelper *)nameOfKey(i));

      delay(SEND_GAP_MS);
    }
  }
}
