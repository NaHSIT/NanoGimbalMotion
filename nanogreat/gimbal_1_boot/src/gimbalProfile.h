// gimbal_1 启动动画工程的机械参数；源自主工程 gimbal_1 配置。
#pragma once

#define GIMBAL_PROFILE 1
#define GIMBAL_PROFILE_NAME "gimbal_1_boot"
#define PAN_GEAR_RATIO 1.0f
#define TILT_GEAR_RATIO 2.488889f
#define TILT_DIRECTION 1.0f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f

// 后向俯仰的最终硬保护由构建参数设置为45°；动画关键帧低于该值。
#define BOOT_TILT_MIN_DEGREES -30.0f
#define BOOT_TILT_MAX_DEGREES 45.0f
