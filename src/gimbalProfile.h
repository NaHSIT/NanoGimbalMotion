// 防止该头文件在一次编译中被重复包含。
#pragma once

// 未指定云台配置时使用通用版本，保持当前固件行为不变。
#ifndef GIMBAL_PROFILE
#define GIMBAL_PROFILE 0
#endif

// 通用版本：作为三个云台副本的校准基准。
#if GIMBAL_PROFILE == 0
#define GIMBAL_PROFILE_NAME "general"
#define PAN_GEAR_RATIO 1.0f
#define TILT_GEAR_RATIO 3.2f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f

// 云台一：后续只修改本组参数，不影响通用版及另外两个云台。
#elif GIMBAL_PROFILE == 1
#define GIMBAL_PROFILE_NAME "gimbal_1"
#define PAN_GEAR_RATIO 1.0f
#define TILT_GEAR_RATIO 3.2f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f

// 云台二：后续只修改本组参数，不影响通用版及另外两个云台。
#elif GIMBAL_PROFILE == 2
#define GIMBAL_PROFILE_NAME "gimbal_2"
#define PAN_GEAR_RATIO 1.0f
#define TILT_GEAR_RATIO 3.2f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f

// 云台三：后续只修改本组参数，不影响通用版及另外两个云台。
#elif GIMBAL_PROFILE == 3
#define GIMBAL_PROFILE_NAME "gimbal_3"
#define PAN_GEAR_RATIO 1.0f
// 实测指令 70° 时机械轴转动 90°，按 70/90 修正原俯仰比例 3.2。
#define TILT_GEAR_RATIO 2.488889f
#define PAN_ANGLE_OFFSET_DEGREES 0.0f
#define TILT_ANGLE_OFFSET_DEGREES 0.0f

#else
#error "GIMBAL_PROFILE 只能是 0、1、2 或 3"
#endif
