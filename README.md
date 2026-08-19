# NanoGimbalMotion

NanoGimbalMotion 是运行在 Arduino Nano 上的三轴步进电机控制固件，用于控制云台水平轴、俯仰轴和直线滑轨轴。固件通过串口接收 ESP8266 发来的文本命令，并使用 TMC2208 驱动器输出 STEP/DIR 控制信号。

## 主要功能

- 控制水平、俯仰和滑轨三个步进电机轴。
- 支持三轴绝对位置控制。
- 支持分别设置三个轴的最大运行速度。
- 支持 TMC2208 的 1/2、1/4、1/8 和 1/16 细分。
- 支持通过一个公共 EN 引脚切换三个驱动器的使能状态。
- 使用完整换行文本帧接收 ESP8266 指令，避免命令拆分导致的错误动作。
- 每次上电将电机当前机械位置定义为软件零点。

## 硬件环境

- Arduino Nano ATmega328P，新版 Bootloader
- TMC2208 步进电机驱动器 × 3
- 两轴云台机构
- 步进电机滑轨
- ESP8266 控制器

## 引脚定义

| 功能 | Arduino Nano 引脚 |
|---|---:|
| 水平轴 STEP | D8 |
| 水平轴 DIR | D7 |
| 俯仰轴 STEP | D6 |
| 俯仰轴 DIR | D5 |
| 滑轨轴 STEP | D4 |
| 滑轨轴 DIR | D3 |
| TMC2208 公共 EN | D12 |
| TMC2208 MS1 | D11 |
| TMC2208 MS2 | D10 |
| 串口 RX | D0 |
| 串口 TX | D1 |

TMC2208 的 EN 引脚为低电平有效。Nano 初始化完成后会将 D12 输出为低电平，使能三个驱动器。

## ESP8266 串口连接

```text
ESP8266 TX  -> Nano D0/RX
ESP8266 RX  <- Nano D1/TX
ESP8266 GND -> Nano GND
```

通信参数：

```text
波特率：57600
数据位：8
停止位：1
校验位：无
```

Arduino Nano 的 TX 是 5 V 电平，ESP8266 的 RX 是 3.3 V 电平。Nano TX 到 ESP8266 RX 之间必须使用电平转换器或电阻分压，避免损坏 ESP8266。

## 串口命令

ESP8266 以换行符 `\n` 结束每条命令。

| 命令 | 参数 | 作用 | 示例 |
|---|---|---|---|
| `p` | 角度 | 设置水平轴绝对目标角度 | `p10.00\n` |
| `t` | 角度 | 设置俯仰轴绝对目标角度 | `t20.00\n` |
| `x` | 毫米 | 设置滑轨轴绝对目标位置 | `x100.00\n` |
| `s` | 度/秒 | 设置水平轴最大角速度 | `s8.00\n` |
| `S` | 度/秒 | 设置俯仰轴最大角速度 | `S5.00\n` |
| `X` | 毫米/秒 | 设置滑轨轴最大线速度 | `X10.00\n` |
| `m` | 2、4、8、16 | 设置 TMC2208 细分 | `m16\n` |
| `e` | 无 | 切换驱动器使能状态 | `e\n` |
| `R` | 无 | 查询驱动器和三轴软件位置 | `R\n` |

Nano 启动后会输出：

```text
OK step mode=16
READY zero=0
```

## TMC2208 细分配置

| 细分 | MS1 | MS2 |
|---:|---:|---:|
| 1/2 | HIGH | LOW |
| 1/4 | LOW | HIGH |
| 1/8 | LOW | LOW |
| 1/16 | HIGH | HIGH |

固件默认使用 1/16 细分。

## 机械换算参数

当前固件的四组独立角度参数位于 `src/gimbalProfile.h`。通用版本保持现有参数，云台一、云台二、云台三初始参数与通用版相同：

```cpp
#define SLIDER_PULLEY_TEETH 36.0f
#define PAN_GEAR_RATIO 1.0f
#define TILT_GEAR_RATIO 3.2f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f
```

- 水平轴按当前机构校准为 1:1。
- 俯仰轴参数与 ESP8266 固件中的角度预补偿配合使用。
- 滑轨按照 36 齿同步轮和 2 mm 同步带齿距进行换算。
- `PAN_ANGLE_OFFSET_DEGREES` 用于修正水平轴零点偏移角度。
- `TILT_ANGLE_OFFSET_DEGREES` 用于修正俯仰轴零点偏移角度。

更换电机、齿轮、同步轮、细分接线或机械结构后，需要重新校准这些参数。

## 云台独立版本

本项目不是四个重复的工程文件夹，而是一个工程目录下的四个 PlatformIO 编译环境。四个版本共用 `src/main.cpp`、`src/panTiltMount.cpp` 和引脚定义；每个版本的差异集中在 `src/gimbalProfile.h` 对应的配置分支中。

nanogreat 已经从本工程完全分离，独立工程位于 [nanogreat](</E:/课程/比赛/物联网/arduino nano/ardno/ardnoctr/nanogreat>)。Nano 主工程不会编译或包含 nanogreat 的开场动画代码；nanogreat 的编译、测试和烧录请进入独立目录执行。

nanogreat 的三套实际启动动画固件位于独立工程下：[`gimbal_1_boot`](nanogreat/gimbal_1_boot)、[`gimbal_2_boot`](nanogreat/gimbal_2_boot)、[`gimbal_3_boot`](nanogreat/gimbal_3_boot)。它们分别对应左、中、右三个云台，上电后自动启动联动轨迹，不等待 ESP8266 发送 `B`；ESP8266 就绪后发送 `F,<pan>,<tilt>`，动画从当前姿态平滑收尾。启动动画不驱动滑轨。

三套动画固件的安全角度统一为 Pan ±30°、Tilt -30°～+45°，其中 +45° 是后向俯仰硬上限，开场关键帧最高 40°。烧录时必须进入对应子目录，使用正式环境，不要烧录 `_test` 环境；完整说明见 [`nanogreat/README.md`](nanogreat/README.md)。

```powershell
cd nanogreat/gimbal_1_boot
pio run -e gimbal_1_boot -t upload --upload-port COM10

cd ../gimbal_2_boot
pio run -e gimbal_2_boot -t upload --upload-port COM10

cd ../gimbal_3_boot
pio run -e gimbal_3_boot -t upload --upload-port COM10
```

| 固件环境 | 用途 | 配置编号 |
|---|---|---:|
| `nanoatmega328` | 通用版本 | 0 |
| `gimbal_1` | 云台一 | 1 |
| `gimbal_2` | 云台二 | 2 |
| `gimbal_3` | 云台三 | 3 |

三个云台版本共用经过验证的电机和串口控制逻辑，仅角度比例与零点偏移参数相互独立。后续校准某个云台时，只修改 `src/gimbalProfile.h` 中对应编号的一组参数。

编译后生成的固件目录位于 `.pio/build/`：

- `.pio/build/nanoatmega328`：通用版本
- `.pio/build/gimbal_1`：云台一
- `.pio/build/gimbal_2`：云台二
- `.pio/build/gimbal_3`：云台三

因此，打开同一个工程后，在 PlatformIO 中选择对应环境即可编译或烧录对应云台版本。

当前校准状态：

- 云台一水平参数与通用版本一致；实测俯仰指令 70° 时机械轴转动 90°，俯仰比例修正为 `2.488889`。
- 云台二水平参数与通用版本一致；实测俯仰指令 70° 时机械轴转动 90°，俯仰比例修正为 `2.488889`。
- 云台三水平参数与通用版本一致。
- 云台三实测俯仰指令 70° 时机械轴转动 90°，因此俯仰比例按 `3.2 × 70 ÷ 90` 修正为 `2.488889`。
- 云台三因俯仰电机机械安装方向相反，独立设置 `TILT_DIRECTION=-1.0f`，只反转云台三俯仰轴的正负方向。

分别编译三个云台版本：

```powershell
pio run -e gimbal_1
pio run -e gimbal_2
pio run -e gimbal_3
```

例如，烧录云台一：

```powershell
pio run -e gimbal_1 -t upload --upload-port COM10
```

## 软件零点

固件不使用霍尔传感器或限位开关。Arduino Nano 每次上电或复位时，都会把电机当时所在的机械位置定义为：

```text
水平位置 = 0
俯仰位置 = 0
滑轨位置 = 0
```

上电前应手动确认三个轴位于安全位置。由于没有物理回零和限位检测，错误的目标位置可能导致机构碰撞。

## PlatformIO 构建

项目使用 PlatformIO，依赖 `AccelStepper`：

```ini
[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328new
framework = arduino
lib_deps =
    waspinator/AccelStepper @ ^1.64
```

编译：

```powershell
pio run
```

自动检测串口并烧录：

```powershell
pio run -t upload
```

指定串口烧录：

```powershell
pio run -t upload --upload-port COM10
```

烧录时应断开连接在 Nano D0/D1 上的 ESP8266，防止串口信号干扰 Bootloader。

## 项目结构

```text
NanoGimbalMotion/
├─ platformio.ini
├─ README.md
└─ src/
   ├─ gimbalProfile.h
   ├─ main.cpp
   ├─ panTiltMount.h
   └─ panTiltMount.cpp
```

ESP8266 控制端源码：[i-am-genius/8266](https://github.com/i-am-genius/8266)
