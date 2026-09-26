#ifndef __TEST_ROBOT_CONFIG_H__
#define __TEST_ROBOT_CONFIG_H__

#include <cstdint>

namespace pyro
{

// =========================================================
// Test Robot 云台 + 发射机构物理参数
// =========================================================

/// Pitch轴最小位置限制 (rad)
constexpr float PITCH_MIN_LIMIT = 0.2970f;

/// Pitch轴最大位置限制 (rad)
constexpr float PITCH_MAX_LIMIT = 0.5866f;

/// 摩擦轮默认目标转速 (rad/s)
constexpr float TEST_FRICTION_DEFAULT_SPEED = 550.0f;

/// 摩擦轮停机速度阈值 (rad/s), 低于此值切零力矩
constexpr float TEST_FRIC_STOP_THRESHOLD = 10.0f;

/// 拨弹盘连发模式目标转速 (rad/s)
constexpr float TRIGGER_CONTINUOUS_RADPS = 20.0f;

/// M2006 拨弹电机减速比
constexpr float TRIGGER_GEAR_RATIO = 36.0f;

// ========== 校准参数 ==========

/// 初始化正转速度 (rad/s), 正转寻找机械死区
constexpr float INIT_FORWARD_RADPS = 3.0f;

/// 堵转检测速度阈值 (rad/s), 低于此值且力矩达到阈值判定为堵转
constexpr float JAM_SPEED_THRESHOLD = 0.3f;

/// 堵转检测力矩阈值 (Nm), 力矩超过此值且速度低判定为堵转
constexpr float JAM_TORQUE_THRESHOLD = 8.0f;

/// 堵转检测时间阈值 (ms), 条件持续超过此值判定为堵转
constexpr uint16_t JAM_DETECT_TIME_MS = 200;

/// 卡弹恢复无力时间 (ms)
constexpr uint16_t JAM_RECOVERY_TIME_MS = 500;

/// 单发完成: 角度误差阈值 (rad)
constexpr float SINGLE_DONE_ANGLE_THRESHOLD = 0.03f;  // 放宽到0.03 rad (约1.7度)

/// 单发完成: 最大超时 (ms)
constexpr uint16_t SINGLE_DONE_TIMEOUT_MS = 400;  // 缩短到400ms，避免长时间等待

/// 单发目标角度: 根据实际弹链测量
/// 常见值：
/// - 8发/圈: PI/4 = 0.7854 rad (45度)
/// - 9发/圈: 2*PI/9 = 0.6981 rad (40度)
/// - 10发/圈: PI/5 = 0.6283 rad (36度)
/// 建议实际测量后调整此值
/// 如果经常双发，减小此值；如果拨不出，增大此值
constexpr float SINGLE_SHOT_ANGLE = 2*PI/9;  // 微调: 约38.9度 (比40度略小)

} // namespace pyro

#endif // __TEST_ROBOT_CONFIG_H__
