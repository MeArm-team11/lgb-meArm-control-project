/*
  按键控制板程序 · Arduino Uno R3 + IIC OLED（0.96" 128x64）
  ==================================================================
  只做三件事，机械臂的动作全部由机械臂端完成：
    1. 扫描 8 个按键，做软件去抖
    2. 按下瞬间通过软串口发 1 个字符给机械臂（长按不重复发，松开不发）
    3. 中文显示绘图模式、示教点数、运行状态、最近动作

  按键分配（一键一符，不用短按长按）
    任务三  D4='N' 循环取放    D5='R' 录制开始/停止
            D6='T' 回放        D7='M' 回中
    任务五  D8='F' 模式切换    D9='D' 开始/暂停/继续
            D10='I' 记点       D11='Q' 取消并回中

  接线（和机械臂只连两根）
    本板 D3(TX)  → 机械臂 D12(RX)
    本板 GND     ↔ 机械臂 GND
    本板 D2(RX)  ← 机械臂 D13(TX)  回传线，暂时不接
    本板 5V      ← 机械臂 5V       可选，仅当舵机独立供电
    本板 D0/D1 空着（USB 下载口），所以烧录不用拔线

  引脚占用
    D2/D3   软串口
    D4-D11  八个按键（一端接引脚，一端接 GND，内部上拉）
    A4/A5   OLED 的 SDA/SCL
    D0/D1   硬件串口，留给电脑调试（115200）
    D12/D13、A0-A3 空着

  屏幕中文怎么来的
    Adafruit 字库只有 ASCII，现成的中文库是 100KB 级别，装不进 32KB Flash，
    所以自己取了 50 个常用字的 16x16 点阵，一个字 32 字节，共约 1.6KB Flash。
    字模在 zh_font.h，由 字模工具/make_zh_font.py 生成，改文案要重新生成。

  库：Adafruit SSD1306（会自动带上 Adafruit GFX）    开发板：Arduino Uno
*/

#define SSD1306_NO_SPLASH   // 去掉库自带开机动画，省约 1KB Flash 给字模

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "zh_font.h"

// ==================== 引脚 ====================
const uint8_t PIN_BTN1 = 4;    // 任务三 循环取放
const uint8_t PIN_BTN2 = 5;    // 任务三 录制
const uint8_t PIN_BTN3 = 6;    // 任务三 回放
const uint8_t PIN_BTN4 = 7;    // 任务三 回中
const uint8_t PIN_BTN5 = 8;    // 任务五 模式
const uint8_t PIN_BTN6 = 9;    // 任务五 开始/暂停/继续
const uint8_t PIN_BTN7 = 10;   // 任务五 记点
const uint8_t PIN_BTN8 = 11;   // 任务五 取消并回中

// 与机械臂通信的软串口：RX, TX。本程序只发送，RX 可以不接
// 对应机械臂端另开的软串口 btnSerial(12, 13)
const uint8_t PIN_ARM_RX = 2;
const uint8_t PIN_ARM_TX = 3;
SoftwareSerial armSerial(PIN_ARM_RX, PIN_ARM_TX);

// ==================== 参数 ====================
const long ARM_BAUD = 9600;    // 与机械臂端 btnSerial.begin(9600) 一致
const long DBG_BAUD = 115200;  // USB 调试口
const unsigned long DEBOUNCE_MS = 30;    // 机械按键抖动 5~20ms，取 30ms
const unsigned long SEND_GAP_MS = 20;    // 发完停一下，避免机械臂还没读完就连发
const uint8_t POINT_TARGET = 5;          // 五点示教

// ==================== OLED ====================
// 0x3C 是绝大多数 0.96 寸模块的地址；背面 A0 焊盘短接的会变成 0x3D
const uint8_t OLED_ADDR = 0x3C;
const uint8_t SCREEN_W = 128;
const uint8_t SCREEN_H = 64;

// 第 3 个参数 &Wire = 用硬件 I2C；第 4 个参数 -1 = 复位脚共用 RESET
// 若编译报"没有匹配的构造函数"，说明装的是 1.x 老版本库，改成 oled(SCREEN_W, SCREEN_H)
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// ==================== 按键表 ====================
// 表驱动：加按键只要在数组里加一行，再到 nameOfKey() 加一个 case
struct ButtonKey {
  uint8_t pin;               // 引脚号
  char    code;              // 按下时发给机械臂的字符
  bool    armReady;          // 机械臂端是否已实现这个字符
  uint8_t lastRaw;           // 上次读到的原始电平
  uint8_t stable;            // 去抖后已确认的电平
  unsigned long lastChange;  // 原始电平上次变化的时刻
};

ButtonKey keys[] = {
  { PIN_BTN1, 'N', true,  HIGH, HIGH, 0 },  // 循环取放
  { PIN_BTN2, 'R', true,  HIGH, HIGH, 0 },  // 录制开始/停止
  { PIN_BTN3, 'T', true,  HIGH, HIGH, 0 },  // 回放
  { PIN_BTN4, 'M', true,  HIGH, HIGH, 0 },  // 回中
  { PIN_BTN5, 'F', false, HIGH, HIGH, 0 },  // 模式切换
  { PIN_BTN6, 'D', false, HIGH, HIGH, 0 },  // 开始/暂停/继续
  { PIN_BTN7, 'I', false, HIGH, HIGH, 0 },  // 记点
  { PIN_BTN8, 'Q', false, HIGH, HIGH, 0 },  // 取消并回中
};

const uint8_t KEY_COUNT = sizeof(keys) / sizeof(keys[0]);

// ==================== 界面文案（全部放 Flash，不占 RAM） ====================
const char S_MODE[]    PROGMEM = "模式 ";
const char S_POINTS[]  PROGMEM = "点数 ";
const char S_COUNT[]   PROGMEM = "次数 ";
const char S_READY[]   PROGMEM = "就绪";
const char S_DRAWING[] PROGMEM = "绘制中";
const char S_PAUSED[]  PROGMEM = "已暂停";
const char S_FULL[]    PROGMEM = "已记满";
const char S_SYSOK[]   PROGMEM = "系统就绪";

const char M_LINE[]    PROGMEM = "直线";
const char M_LETTER[]  PROGMEM = "字母";
const char M_POLY[]    PROGMEM = "五点折线";
const char M_CURVE[]   PROGMEM = "五点曲线";
PGM_P const MODES[4]   PROGMEM = { M_LINE, M_LETTER, M_POLY, M_CURVE };

const char N_LOOP[]    PROGMEM = "循环取放";
const char N_REC[]     PROGMEM = "录制";
const char N_PLAY[]    PROGMEM = "回放";
const char N_HOME[]    PROGMEM = "回中";
const char N_MODE[]    PROGMEM = "模式";
const char N_START[]   PROGMEM = "开始";
const char N_PAUSE[]   PROGMEM = "暂停";
const char N_RESUME[]  PROGMEM = "继续";
const char N_TEACH[]   PROGMEM = "记点";
const char N_CANCEL[]  PROGMEM = "取消";

// ==================== 屏幕状态 ====================
unsigned int pressCount = 0;    // 累计按键次数，用来确认"一次按下只触发一次"
PGM_P lastName = S_READY;       // 最近一次动作的中文名（2 字节 RAM）

// 下面是"本地镜像"：串口是单向的，按钮板不知道机械臂的真实状态，
// 只能按"按了什么键"推算。要和机械臂一致需接回传线（机械臂 D13 → 本板 D2）。
uint8_t modeIndex = 0;          // 0=直线 1=字母 2=五点折线 3=五点曲线
uint8_t pointCount = 0;         // 已记录示教点数
uint8_t runState = 0;           // 0=就绪 1=绘制中 2=已暂停
const uint8_t MODE_COUNT = 4;

// ==================== 中文显示 ====================
// 在字库里查一个汉字，返回序号；找不到返回 -1
int zhIndexOfP(PGM_P p) {
  uint8_t b0 = pgm_read_byte(p);
  uint8_t b1 = pgm_read_byte(p + 1);
  uint8_t b2 = pgm_read_byte(p + 2);
  for (int i = 0; i < ZH_COUNT; i++) {
    if (pgm_read_byte(&zhChars[i * 3 + 0]) == b0 &&
        pgm_read_byte(&zhChars[i * 3 + 1]) == b1 &&
        pgm_read_byte(&zhChars[i * 3 + 2]) == b2) {
      return i;
    }
  }
  return -1;
}

// 画一行中英混排文字，返回画完后的 x 坐标
// 汉字走 16x16 点阵，ASCII（数字、斜杠）走内置 6x8 字库
int drawZh(int x, int y, PGM_P s) {
  while (true) {
    uint8_t c = pgm_read_byte(s);
    if (c == 0) {
      break;
    }

    if (c >= 0xE0) {
      // UTF-8 三字节 = 一个汉字
      int idx = zhIndexOfP(s);
      if (idx >= 0) {
        oled.drawBitmap(x, y, zhFont[idx], 16, 16, SSD1306_WHITE);
      } else {
        oled.drawRect(x, y, 15, 15, SSD1306_WHITE);   // 缺字画方框，一眼能看出来
      }
      x += 16;
      s += 3;
    } else {
      // ASCII 往下挪 4 像素，才和 16 高的汉字对齐
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(x, y + 4);
      oled.write(c);
      x += 6;
      s += 1;
    }
  }
  return x;
}

// 画一个 ASCII 字符，返回画完后的 x
int drawCh(int x, int y, char c) {
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(x, y + 4);
  oled.write(c);
  return x + 6;
}

// 画一个非负整数，返回画完后的 x
// 用 Print 自带的数字输出，省掉 sprintf 那一大坨库代码
int drawInt(int x, int y, int v) {
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(x, y + 4);
  oled.print(v);

  int digits = 1;
  while (v >= 10) {
    v /= 10;
    digits++;
  }
  return x + 6 * digits;
}

// ==================== 画三行状态 ====================
// 模式 XXXX / 点数 3/5 就绪 / 记点 次数 12
void drawScreen() {
  oled.clearDisplay();

  int x = drawZh(0, 0, S_MODE);
  drawZh(x, 0, (PGM_P)pgm_read_word(&MODES[modeIndex]));

  x = drawZh(0, 22, S_POINTS);
  x = drawInt(x, 22, pointCount);
  x = drawCh(x, 22, '/');
  x = drawInt(x, 22, POINT_TARGET);
  x += 6;

  // 只有"五点"两档才谈得上点数满不满
  bool isTeachMode = (modeIndex >= 2);
  PGM_P st;
  if (isTeachMode && pointCount >= POINT_TARGET && runState == 0) {
    st = S_FULL;                             // 记满 5 点：再按记点键就是清空重记
  } else if (runState == 1) {
    st = S_DRAWING;
  } else if (runState == 2) {
    st = S_PAUSED;
  } else {
    st = S_READY;
  }
  drawZh(x, 22, st);

  x = drawZh(0, 44, lastName);
  x += 6;
  x = drawZh(x, 44, S_COUNT);
  drawInt(x, 44, pressCount);

  // display() 要推 1024 字节整帧，是阻塞操作，所以只在状态变化时调用
  oled.display();
}

// ==================== 按键 → 动作名 ====================
PGM_P nameOfKey(uint8_t idx) {
  switch (idx) {
    case 0:  return N_LOOP;      // N
    case 1:  return N_REC;       // R
    case 2:  return N_PLAY;      // T
    case 3:  return N_HOME;      // M
    case 4:  return N_MODE;      // F
    case 5:  return N_START;     // D（下面按状态改成 暂停/继续）
    case 6:  return N_TEACH;     // I
    case 7:  return N_CANCEL;    // Q
    default: return S_READY;
  }
}

// ==================== 更新本地镜像状态 ====================
// 只按"按了哪个键"推算，用于屏幕显示；
// 动作是否真的执行、要不要拒绝，全部由机械臂端决定
void updateLocalState(char code) {
  switch (code) {
    case 'F':
      modeIndex = (modeIndex + 1) % MODE_COUNT;   // 切模式不清空示教点
      break;

    case 'I':
      // 记满 5 点后再按一次 = 清空重记（现场记错点的唯一补救手段）
      if (pointCount >= POINT_TARGET) {
        pointCount = 1;
      } else {
        pointCount++;
      }
      runState = 0;
      break;

    case 'D':
      // 开始 → 暂停 → 继续 → 暂停 …… 一个键循环
      if (runState == 0) {
        runState = 1;
        lastName = N_START;
      } else if (runState == 1) {
        runState = 2;
        lastName = N_PAUSE;
      } else {
        runState = 1;
        lastName = N_RESUME;
      }
      break;

    case 'Q':
      runState = 0;    // 取消并回中：示教点保留
      break;

    default:
      break;           // 任务三的 N/R/T/M 不改变任务五状态
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

  Wire.begin();

  // begin() 的返回值一定要看：false 就是接线或地址有问题，不用盲猜
  bool oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  if (oledOk) {
    // 默认 I2C 100kHz 刷整屏要 ~90ms，提到 400kHz 只要 ~23ms。
    // 这直接影响"短按会不会丢"。
    Wire.setClock(400000);

    oled.clearDisplay();
    drawZh(32, 24, S_SYSOK);   // 4 个汉字 = 64 像素，居中
    oled.display();
  }

  Serial.println(F("按键控制板就绪（Uno + OLED 中文版，8 键）"));
  Serial.println(F("任务三: 1=N 2=R 3=T 4=M"));
  Serial.println(F("任务五: 5=F 模式 / 6=D 开始暂停继续 / 7=I 记点 / 8=Q 取消回中"));
  if (oledOk) {
    Serial.println(F("OLED 初始化成功，地址 0x3C"));
  } else {
    Serial.println(F("OLED 初始化失败：检查 VCC/GND、SDA/SCL 和地址 0x3C/0x3D"));
  }
  Serial.println(F("提醒：F/D/I/Q 机械臂端还没实现，按了不会有反应"));

  delay(1000);   // 让"系统就绪"停 1 秒
  drawScreen();
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
      // 先发字符再刷屏：软串口发送会短暂关中断，刷屏是长阻塞，两件事不重叠
      armSerial.write(keys[i].code);

      lastName = nameOfKey(i);
      pressCount++;
      updateLocalState(keys[i].code);   // 里面可能把 lastName 改成 暂停/继续
      drawScreen();                     // 只在按键事件后刷新，平时不刷

      Serial.print(F("按键"));
      Serial.print(i + 1);
      Serial.print(F(" 已按下，发送 "));
      Serial.print(keys[i].code);
      if (keys[i].armReady) {
        Serial.println();
      } else {
        Serial.println(F("  （机械臂端还没实现这个字符）"));
      }

      delay(SEND_GAP_MS);
    }
  }
}
