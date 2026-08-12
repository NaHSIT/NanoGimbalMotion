// 引入云台与滑轨控制模块的公开接口。
#include "panTiltMount.h"

// Arduino 上电或复位后只执行一次 setup()。
void setup() {
    // 初始化串口、TMC2208 控制引脚、三轴步进对象及软件零点。
    initPanTilt();
}

// Arduino 初始化完成后会持续、反复调用 loop()。
void loop() {
    // 处理 ESP8266 指令，并持续为步进驱动器生成 STEP 脉冲。
    mainLoop();
}
