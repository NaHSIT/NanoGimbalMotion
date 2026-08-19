# nanogreat 独立工程

本目录是从 NanoGimbalMotion 主工程中分离出的 nanogreat 固件工程，专门用于 nanogreat 的非阻塞开场动画和三轴控制。

## 与 NanoGimbalMotion 的区别

nanogreat 与 Nano 云台通用版、云台一、云台二、云台三是不同工程，不再共用主工程入口，也不会参与 Nano 四个云台环境的编译。

| 工程 | 目录 | 用途 |
|---|---|---|
| NanoGimbalMotion | 上级目录 | 通用版、云台一、云台二、云台三 |
| nanogreat | 当前目录 | nanogreat 开场动画和控制固件 |

nanogreat 的源码位于当前目录的 `src/`，专用的 `BootMotion` 库位于当前目录的 `lib/BootMotion/`。主工程的 `src/`、`lib/` 不再包含 nanogreat 源码。

## 三个带启动动画的云台版本

为避免影响上级 Nano 主工程，基于原始 gimbal_1、gimbal_2、gimbal_3 校准参数分别复制了三个独立工程：

| 工程目录 | 对应原始版本 | 关键帧角色 |
|---|---|---|
| `gimbal_1_boot` | gimbal_1 | 左侧展开轨迹 |
| `gimbal_2_boot` | gimbal_2 | 中间抬头轨迹 |
| `gimbal_3_boot` | gimbal_3 | 右侧镜像轨迹 |

三个工程都在上电后自动进入 `OPENING`，不等待 ESP8266 的 `B`；收到 `F,<pan>,<tilt>` 后从当前姿态平滑收尾。三个版本各自包含完整的 `src/`、`lib/BootMotion/` 和 `platformio.ini`，并通过 `../lib/AccelStepper` 共用离线依赖，不会修改主工程中的 `gimbal_1`、`gimbal_2`、`gimbal_3` 环境。

安全边界统一为 Pan ±30°，俯仰下限 -30°、后向上限 +45°；开场关键帧最高 40°，为 45° 后向硬限留出余量。启动动画不驱动滑轨，收到 `F` 后两轴同步收尾。

分别构建：

```powershell
cd nanogreat/gimbal_1_boot
pio run -e gimbal_1_boot

cd ../gimbal_2_boot
pio run -e gimbal_2_boot

cd ../gimbal_3_boot
pio run -e gimbal_3_boot
```

烧录时使用对应目录的正式环境，不要烧录 `_test` 环境。

## 编译

在当前 `nanogreat` 目录执行：

```powershell
pio run -e nanogreat
```

编译轨迹自检：

```powershell
pio run -e nanogreat_test
```

烧录：

```powershell
pio run -e nanogreat -t upload --upload-port COM10
```

请根据设备管理器中的实际串口替换 `COM10`。

## Git 提交区分

nanogreat 的新增、修改和修复应在提交说明中明确使用 `nanogreat:` 前缀；Nano 云台主工程使用 `nano:` 前缀。例如：

```text
nanogreat: 增加开场动画超时保护
nano: 校准云台三俯仰角度
```
