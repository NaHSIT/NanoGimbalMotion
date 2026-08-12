// 引入本模块的引脚、机械参数和公开接口定义。
#include "panTiltMount.h"

// 引入单轴步进电机控制类。
#include <AccelStepper.h>

// 引入多轴同步到达目标位置的控制类。
#include <MultiStepper.h>

// 引入 atof() 等字符串数值转换函数。
#include <stdlib.h>

// 匿名命名空间使内部变量和函数仅在本源文件中可见。
namespace {

// 创建水平轴步进对象，接口模式为 STEP/DIR 驱动器。
AccelStepper pan(AccelStepper::DRIVER, PIN_STEP_PAN, PIN_DIRECTION_PAN);

// 创建俯仰轴步进对象，接口模式为 STEP/DIR 驱动器。
AccelStepper tilt(AccelStepper::DRIVER, PIN_STEP_TILT, PIN_DIRECTION_TILT);

// 创建滑轨轴步进对象，接口模式为 STEP/DIR 驱动器。
AccelStepper slider(AccelStepper::DRIVER, PIN_STEP_SLIDER, PIN_DIRECTION_SLIDER);

// 创建三轴协调控制器，使参与运动的轴同时到达各自目标位置。
MultiStepper coordinated;

// 保存三轴绝对目标步数：0=水平，1=俯仰，2=滑轨。
long targets[3] = {0, 0, 0};

// 保存当前外部 STEP 输入细分，默认使用 1/16 细分。
int stepMode = SIXTEENTH_STEP;

// 保存驱动器使能状态；true 表示当前驱动器已使能。
bool driversEnabled = true;

// 标记滑轨是否仍在执行最近一次绝对位置命令。
bool sliderMotionPending = false;

// 保存最近一次滑轨命令的目标毫米值。
float sliderTargetMm = 0.0f;

// 保存最近一次滑轨命令的目标 STEP 位置。
long sliderTargetSteps = 0;

// 标记当前是否已经收到一条文本命令的命令字符。
bool textCommandPending = false;

// 保存正在接收的文本命令字符，例如 p、t、x。
char pendingCommand = 0;

// 保存文本命令后面的数值字符串，末尾预留一个字符串结束符。
char pendingValue[20] = {0};

// 记录 pendingValue 中已经写入的有效字符数量。
uint8_t pendingValueLength = 0;

// 水平轴最大角速度，单位为度/秒。
float panMaxDegPerSecond = 18.0f;

// 俯仰轴最大角速度，单位为度/秒。
float tiltMaxDegPerSecond = 10.0f;

// 滑轨轴最大线速度，单位为毫米/秒。
float sliderMaxMmPerSecond = 20.0f;

// 水平轴每旋转一度需要的 STEP 脉冲数，由细分和传动比动态计算。
float panStepsPerDegree;

// 俯仰轴每旋转一度需要的 STEP 脉冲数，由细分和校准系数动态计算。
float tiltStepsPerDegree;

// 滑轨每移动一毫米需要的 STEP 脉冲数，由细分、同步轮和齿距动态计算。
float sliderStepsPerMm;

// 将水平轴角度转换为对应的 STEP 脉冲数。
float panToSteps(float degrees) {
    // 角度乘以每度脉冲数，保留浮点结果供速度和位置换算使用。
    return degrees * panStepsPerDegree;
}

// 将俯仰轴角度转换为对应的 STEP 脉冲数。
float tiltToSteps(float degrees) {
    // 角度乘以每度脉冲数，保留浮点结果供速度和位置换算使用。
    return degrees * tiltStepsPerDegree;
}

// 将滑轨毫米位置转换为整数 STEP 脉冲数。
long sliderToSteps(float mm) {
    // 步进脉冲必须为整数，因此对计算结果进行四舍五入。
    return lroundf(mm * sliderStepsPerMm);
}

// 判断任一轴是否尚未到达当前软件目标。
bool axesMoving() {
    return pan.currentPosition() != targets[0] ||
           tilt.currentPosition() != targets[1] ||
           slider.currentPosition() != targets[2];
}

// 向 ESP8266 返回单行、可精确匹配的 Nano 通信健康状态。
void printNanoHealth() {
    Serial.print(F("NANO_OK v=2 en="));
    Serial.print(driversEnabled ? 1 : 0);
    Serial.print(F(" moving="));
    Serial.print(axesMoving() ? 1 : 0);
    Serial.print(F(" slider_steps="));
    Serial.println(slider.currentPosition());
}

// 向 ESP8266 返回最近一次滑轨绝对位置命令的软件完成回执。
void printSliderDone() {
    Serial.print(F("SLIDER_DONE target_mm="));
    Serial.print(sliderTargetMm, 2);
    Serial.print(F(" pos_steps="));
    Serial.println(slider.currentPosition());
}

// 向 ESP8266 输出当前驱动器状态及三轴软件位置。
void printStatus() {
    // 输出状态报告起始行。
    Serial.println(F("Status"));

    // 输出使能状态字段名称。
    Serial.print(F("Enable state: "));

    // 以 1 或 0 输出驱动器当前是否使能。
    Serial.println(driversEnabled ? 1 : 0);

    // 输出水平轴当前位置，单位为内部步数。
    Serial.print(F("POS pan="));
    Serial.print(pan.currentPosition());

    // 输出俯仰轴当前位置，单位为内部步数。
    Serial.print(F(" tilt="));
    Serial.print(tilt.currentPosition());

    // 输出滑轨轴当前位置，单位为内部步数，并结束本行。
    Serial.print(F(" slider="));
    Serial.println(slider.currentPosition());
}

// 根据物理速度设置更新三个 AccelStepper 对象的最大步速。
void applyMaxSpeeds() {
    // 将水平角速度转换为步/秒；fabs() 保证最大速度始终为正值。
    pan.setMaxSpeed(fabs(panToSteps(panMaxDegPerSecond)));

    // 将俯仰角速度转换为步/秒；fabs() 保证最大速度始终为正值。
    tilt.setMaxSpeed(fabs(tiltToSteps(tiltMaxDegPerSecond)));

    // 将滑轨线速度转换为步/秒；fabs() 保证最大速度始终为正值。
    slider.setMaxSpeed(fabs(sliderToSteps(sliderMaxMmPerSecond)));
}

// 设置 TMC2208 独立模式细分，并重新计算所有单位换算参数。
void setStepMode(int mode) {
    // 只允许 TMC2208 当前接线支持的 1/2、1/4、1/8 和 1/16 细分。
    if (mode != HALF_STEP && mode != QUARTER_STEP &&
        mode != EIGHTH_STEP && mode != SIXTEENTH_STEP) {
        // 向控制端报告非法细分值。
        Serial.println(F("ERR step mode: use 2,4,8,16"));

        // 非法参数不改变当前细分和运动参数。
        return;
    }

    // TMC2208 独立模式真值表：00=1/8，10=1/2，01=1/4，11=1/16。
    // 当模式为 1/2 或 1/16 时，将 MS1 输出为高电平，否则输出低电平。
    digitalWrite(PIN_MS1, mode == HALF_STEP || mode == SIXTEENTH_STEP);

    // 当模式为 1/4 或 1/16 时，将 MS2 输出为高电平，否则输出低电平。
    digitalWrite(PIN_MS2, mode == QUARTER_STEP || mode == SIXTEENTH_STEP);

    // 保存已经应用到驱动器的细分值。
    stepMode = mode;

    // 电机每圈 200 整步；结合细分和水平传动比计算每度脉冲数。
    panStepsPerDegree = (200.0f * stepMode * PAN_GEAR_RATIO) / 360.0f;

    // 电机每圈 200 整步；结合细分和俯仰校准系数计算每度脉冲数。
    tiltStepsPerDegree = (200.0f * stepMode * TILT_GEAR_RATIO) / 360.0f;

    // 同步带齿距为 2 mm，计算滑轨每毫米需要的脉冲数。
    sliderStepsPerMm = (200.0f * stepMode) / (SLIDER_PULLEY_TEETH * 2.0f);

    // 细分改变后，重新换算并应用三个轴的最大步速。
    applyMaxSpeeds();

    // 向 ESP8266 报告当前细分设置。
    Serial.print(F("OK step mode="));
    Serial.println(stepMode);
}

// 将 targets 数组中的三轴绝对目标提交给 MultiStepper。
void applyTargets() {
    // 根据各轴剩余距离计算协调速度，并设置三个轴的新目标位置。
    coordinated.moveTo(targets);
}

// 单独更新水平轴绝对角度目标，不覆盖另外两个轴已有目标。
void setPanTarget(float degrees) {
    // 将水平角度转换为整数目标步数并写入 targets[0]。
    targets[0] = lroundf(panToSteps(degrees));

    // 重新提交包含三个轴当前目标值的目标数组。
    applyTargets();
}

// 单独更新俯仰轴绝对角度目标，不覆盖另外两个轴已有目标。
void setTiltTarget(float degrees) {
    // 将俯仰角度转换为整数目标步数并写入 targets[1]。
    targets[1] = lroundf(tiltToSteps(degrees));

    // 重新提交包含三个轴当前目标值的目标数组。
    applyTargets();
}

// 单独更新滑轨轴绝对毫米目标，不覆盖另外两个轴已有目标。
void setSliderTarget(float mm) {
    // 保存原始毫米目标，供运动完成回执使用。
    sliderTargetMm = mm;

    // 将毫米位置转换为整数目标步数并写入 targets[2]。
    sliderTargetSteps = sliderToSteps(mm);
    targets[2] = sliderTargetSteps;

    // 只有当前位置与目标不同时才等待主循环发送完成回执。
    sliderMotionPending = slider.currentPosition() != sliderTargetSteps;

    // 重新提交包含三个轴当前目标值的目标数组。
    applyTargets();

    // 重复发送当前位置时也必须立即返回对应命令的完成回执。
    if (!sliderMotionPending) {
        printSliderDone();
    }
}

// 执行一条已经完整接收的文本命令。
void executeTextCommand(char command, const char* value) {
    // 将命令参数从字符串转换为浮点数；无参数命令会得到 0。
    float number = atof(value);

    // 根据首字符选择与原 ESP8266 协议对应的操作。
    switch (command) {
        // e：翻转三个 TMC2208 的公共使能状态。
        case 'e':
            // 反转软件记录的使能状态。
            driversEnabled = !driversEnabled;

            // TMC2208 EN 为低有效：使能输出 LOW，关闭输出 HIGH。
            digitalWrite(PIN_ENABLE, driversEnabled ? LOW : HIGH);

            // 返回 ESP8266 启动同步逻辑能够识别的状态文本。
            Serial.println(driversEnabled ? F("Enabled") : F("Disabled"));
            break;

        // m：设置 TMC2208 的外部 STEP 输入细分。
        case 'm':
            setStepMode(static_cast<int>(number));
            break;

        // p：设置水平轴绝对目标角度。
        case 'p':
            setPanTarget(number);
            break;

        // t：设置俯仰轴绝对目标角度。
        case 't':
            setTiltTarget(number);
            break;

        // x：设置滑轨轴绝对目标位置，单位为毫米。
        case 'x':
            setSliderTarget(number);
            break;

        // s：设置水平轴最大角速度，单位为度/秒。
        case 's':
            panMaxDegPerSecond = number;
            applyMaxSpeeds();
            break;

        // S：设置俯仰轴最大角速度，单位为度/秒。
        case 'S':
            tiltMaxDegPerSecond = number;
            applyMaxSpeeds();
            break;

        // X：设置滑轨轴最大线速度，单位为毫米/秒。
        case 'X':
            sliderMaxMmPerSecond = number;
            applyMaxSpeeds();
            break;

        // Q：返回单行 Nano 通信健康状态，供 ESP8266 自检精确匹配。
        case 'Q':
            printNanoHealth();
            break;

        // R：输出当前驱动器和三轴状态。
        case 'R':
            printStatus();
            break;

        // 未定义字符保持静默，不执行任何操作。
        default:
            break;
    }
}

// 结束当前文本命令的接收并执行该命令。
void finishTextCommand() {
    // 在有效参数末尾写入 C 字符串结束符。
    pendingValue[pendingValueLength] = '\0';

    // 将完整命令字符和参数字符串交给命令执行器。
    executeTextCommand(pendingCommand, pendingValue);

    // 清除“正在接收命令”标志，允许解析下一条命令。
    textCommandPending = false;

    // 清空暂存的命令字符。
    pendingCommand = 0;

    // 将参数长度复位为 0。
    pendingValueLength = 0;

    // 清空参数缓冲区的首字节，使其立即成为空字符串。
    pendingValue[0] = '\0';
}

// 非阻塞地接收 ESP8266 文本命令。
void serviceSerial() {
    // 处理当前已经进入 Nano 硬件串口缓冲区的所有字节。
    while (Serial.available()) {
        // 尚未开始文本命令时，当前字节就是命令字符。
        if (!textCommandPending) {
            // 文本协议的第一个字节就是命令字符。
            pendingCommand = static_cast<char>(Serial.read());

            // 标记已经开始接收该命令的参数部分。
            textCommandPending = true;

            // 新命令开始时清空参数长度。
            pendingValueLength = 0;

            // 返回循环顶部，继续读取该命令后面的参数字符。
            continue;
        }

        // 读取文本命令的下一个参数字符或结束字符。
        char c = static_cast<char>(Serial.read());

        // ESP8266 使用换行结束命令，同时支持回车结束符。
        if (c == '\n' || c == '\r') {
            // 执行当前已经完整接收的命令。
            finishTextCommand();

            // 继续解析串口缓冲区中的下一条命令。
            continue;
        }

        // 只在参数缓冲区仍有空间时保存字符，防止数组越界。
        if (pendingValueLength < sizeof(pendingValue) - 1) {
            // 将当前参数字符追加到缓冲区，并将有效长度加一。
            pendingValue[pendingValueLength++] = c;
        }
    }
}

// 结束匿名命名空间。
}  // namespace

// 初始化串口、控制引脚、运动参数和三轴软件零点。
void initPanTilt() {
    // 启动 Nano 硬件串口，与 ESP8266 使用相同波特率。
    Serial.begin(BAUD_RATE);

    // 将 TMC2208 的 MS1 细分选择引脚设置为输出。
    pinMode(PIN_MS1, OUTPUT);

    // 将 TMC2208 的 MS2 细分选择引脚设置为输出。
    pinMode(PIN_MS2, OUTPUT);

    // 将三个驱动器共用的 EN 使能引脚设置为输出。
    pinMode(PIN_ENABLE, OUTPUT);

    // 将水平轴 STEP 引脚设置为输出。
    pinMode(PIN_STEP_PAN, OUTPUT);

    // 将水平轴 DIR 引脚设置为输出。
    pinMode(PIN_DIRECTION_PAN, OUTPUT);

    // 将俯仰轴 STEP 引脚设置为输出。
    pinMode(PIN_STEP_TILT, OUTPUT);

    // 将俯仰轴 DIR 引脚设置为输出。
    pinMode(PIN_DIRECTION_TILT, OUTPUT);

    // 将滑轨轴 STEP 引脚设置为输出。
    pinMode(PIN_STEP_SLIDER, OUTPUT);

    // 将滑轨轴 DIR 引脚设置为输出。
    pinMode(PIN_DIRECTION_SLIDER, OUTPUT);

    // 每次上电都将当前机械位置定义为水平轴软件零点。
    pan.setCurrentPosition(0);

    // 每次上电都将当前机械位置定义为俯仰轴软件零点。
    tilt.setCurrentPosition(0);

    // 每次上电都将当前机械位置定义为滑轨轴软件零点。
    slider.setCurrentPosition(0);

    // 同步清零三个轴的绝对目标步数。
    targets[0] = targets[1] = targets[2] = 0;

    // 默认将 TMC2208 设置为 1/16 细分，并完成单位换算。
    setStepMode(SIXTEENTH_STEP);

    // 按 targets[0] 的顺序将水平轴加入协调控制器。
    coordinated.addStepper(pan);

    // 按 targets[1] 的顺序将俯仰轴加入协调控制器。
    coordinated.addStepper(tilt);

    // 按 targets[2] 的顺序将滑轨轴加入协调控制器。
    coordinated.addStepper(slider);

    // TMC2208 EN 为低有效，上电初始化完成后使能三个驱动器。
    digitalWrite(PIN_ENABLE, LOW);

    // 使软件状态与实际 EN 输出保持一致。
    driversEnabled = true;

    // 通知 ESP8266：Nano 已初始化，三轴软件位置均从 0 开始。
    Serial.println(F("READY zero=0"));
}

// 执行一次主循环任务；该函数必须被 Arduino loop() 高频调用。
void mainLoop() {
    // 非阻塞处理当前可用的 ESP8266 串口数据。
    serviceSerial();

    // 持续推进三个轴到各自绝对目标位置。
    coordinated.run();

    // 滑轨首次达到最近目标时只发送一次软件运动完成回执。
    if (
        sliderMotionPending &&
        slider.currentPosition() == sliderTargetSteps
    ) {
        sliderMotionPending = false;
        printSliderDone();
    }
}
