/*
  I2C 地址扫描器 · 烧给按钮板用，不接机械臂
  ------------------------------------------------------------------
  用途：OLED 不亮、或者不确定这块屏的 I2C 地址是 0x3C 还是 0x3D 时，
        先烧这个程序，打开串口监视器（115200）看结果。

  接线：OLED 只接 VCC(5V) / GND / SDA(A4) / SCL(A5)，四根线即可。
        不接软件串口，不接机械臂。

  正常输出：
        发现设备地址: 0x3C
  如果是 0x3D，把主程序里的 OLED_ADDR 改成 0x3D 就行。

  如果一个设备都扫不到，按这个顺序查：
        1. VCC 有没有接 5V、GND 有没有接
        2. SDA 是不是接了 A4、SCL 是不是接了 A5（接反了扫不到）
        3. 模块丝印顺序是不是 VCC/GND/SCL/SDA，别按顺序插
        4. 杜邦线虚接、面包板那一排没导通
*/

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Serial.println(F("I2C 扫描开始..."));

  uint8_t found = 0;

  // I2C 7 位地址范围是 1~126，逐个发一次空传输，
  // 有设备应答（ACK）就说明这个地址上有东西
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("发现设备地址: 0x"));
      if (addr < 16) {
        Serial.print('0');   // 补零，凑成两位好看
      }
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("没有发现任何 I2C 设备：先查接线，再查地址"));
  }

  Serial.print(F("扫描结束，共发现 "));
  Serial.print(found);
  Serial.println(F(" 个设备"));
}

void loop() {
  // 只扫一次，什么都不做
}
