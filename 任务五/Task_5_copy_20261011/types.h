#ifndef TASK5_TYPES_H
#define TASK5_TYPES_H

// ==================== 任务五 数据类型定义 ====================
// 这些结构体放在头文件里，而不是 .ino 里。
// 原因：Arduino IDE 会把自动生成的函数原型插到 .ino 文件靠前的位置，
// 如果结构体也写在 .ino 里，原型可能排在结构体定义之前，导致
// "ActionData was not declared in this scope" 之类的编译错误。
// 头文件不受原型插入影响，从上面的 #include 位置起就可见，顺序永远正确。

// 记录一个位置的三个舵机角度
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

// 录制时保存的一帧：四个舵机角度
// 不保存时间戳：帧是每 recordStep 毫秒存一帧的，
// 第 i 帧的时刻恒等于 i * recordStep，存下来纯属浪费 SRAM。
// 去掉 timeMs 后每帧 4 字节，80 帧只需 320 字节（原来要 640 字节）。
struct RecordFrame {
  byte b;
  byte f;
  byte r;
  byte c;
};

#endif
