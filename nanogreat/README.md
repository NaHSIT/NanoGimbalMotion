# nanogreat 独立工程

本目录是从 NanoGimbalMotion 主工程中分离出的 nanogreat 固件工程，专门用于 nanogreat 的非阻塞开场动画和三轴控制。

## 与 NanoGimbalMotion 的区别

nanogreat 与 Nano 云台通用版、云台一、云台二、云台三是不同工程，不再共用主工程入口，也不会参与 Nano 四个云台环境的编译。

| 工程 | 目录 | 用途 |
|---|---|---|
| NanoGimbalMotion | 上级目录 | 通用版、云台一、云台二、云台三 |
| nanogreat | 当前目录 | nanogreat 开场动画和控制固件 |

nanogreat 的源码位于当前目录的 `src/`，专用的 `BootMotion` 库位于当前目录的 `lib/BootMotion/`。主工程的 `src/`、`lib/` 不再包含 nanogreat 源码。

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

