#ifndef __TEST_ROBOT_CONFIG_H__
#define __TEST_ROBOT_CONFIG_H__

#include <cstdint>

namespace pyro
{

// =========================================================
// Test Robot 云台 + 发射机构物理参数
// =========================================================

/// 摩擦轮默认目标转速 (rad/s)
constexpr float TEST_FRICTION_DEFAULT_SPEED = 500.0f;

/// 摩擦轮停机速度阈值 (rad/s), 低于此值切零力矩
constexpr float TEST_FRIC_STOP_THRESHOLD = 10.0f;

/// 拨弹盘连发模式目标转速 (rad/s)
constexpr float TRIGGER_CONTINUOUS_RADPS = 10.0f;

/// M2006 拨弹电机减速比
constexpr float TRIGGER_GEAR_RATIO = 36.0f;

// ========== 校准参数 ==========

/// 校准反转速度 (rad/s), 高速反转寻找机械死区
constexpr float CALI_REVERSE_RADPS = -3.0f;

/// 校准正转目标角度 (rad), 从死区正转 C 角度确保脱离
constexpr float CALI_FORWARD_ANGLE = 0.2f;

/// 堵转检测速度误差阈值 (50% = 目标速度的 50%)
constexpr float CALI_BLOCK_THRESHOLD = 0.5f;

/// 堵转检测时间阈值 (ms), 条件持续超过此值判定为堵转
constexpr uint16_t CALI_BLOCK_TIME_MS = 1000;

/// 校准完成角度误差阈值 (rad)
constexpr float CALI_DONE_ANGLE_THRESHOLD = 0.01f;

/// 单发完成: 角度误差阈值 (rad)
constexpr float SINGLE_DONE_ANGLE_THRESHOLD = 0.003f;

/// 单发完成: 最大超时 (ms)
constexpr uint16_t SINGLE_DONE_TIMEOUT_MS = 500;

/// 单发目标角度: PI/4 = 45° per shot (8发/圈)
constexpr float SINGLE_SHOT_ANGLE = 0.785398163f; // PI / 4

} // namespace pyro

#endif // __TEST_ROBOT_CONFIG_H__
