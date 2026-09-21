#ifndef __LARGE_LAUNCHER_CONFIG_H__
#define __LARGE_LAUNCHER_CONFIG_H__

#include <cstdint>

namespace pyro
{

// ========== 摩擦轮参数 ==========
/// 摩擦轮半径 (m) - 三个摩擦轮
constexpr float FRIC1_RADIUS = 0.04f;  // 第一组摩擦轮半径
constexpr float FRIC2_RADIUS = 0.04f;  // 第二组摩擦轮半径
constexpr float FRIC3_RADIUS = 0.04f;  // 第三组摩擦轮半径

/// 摩擦轮停机速度阈值 (rad/s), 低于此值切零力矩
constexpr float FRIC_STOP_THRESHOLD = 10.0f;

/// 防卡弹反转力矩
constexpr float FRIC_ANTI_JAM_REVERSE_TORQUE = 4.0f;

// ========== 拨弹盘参数 (DM4310) ==========
/// 拨弹盘编码器偏移 (根据实际机械结构调整)
constexpr float TRIGGER_OFFSET = 0.1f;

/// 拨弹盘最小推进角度 (rad)
constexpr float TRIGGER_PRESET_MIN_ADVANCE_RAD = 0.15f;

/// 拨弹盘预设变形阈值 (rad)
constexpr float TRIGGER_PRESET_DEFORM_THRESHOLD_RAD = 0.15f;

/// 拨弹盘定位阈值 (rad)
constexpr float TRIGGER_LOCATED_THRESHOLD_RAD = 0.1f;

/// 拨弹盘进弹方向 (1.0 为正向, -1.0 为反向)
constexpr float TRIGGER_FEED_DIR = -1.0f;

/// 单发目标角度: PI/3 = 60° per shot (6发/圈)
constexpr float SINGLE_SHOT_ANGLE = PI / 3.0f;

// ========== 校准参数 ==========
/// 校准反转速度 (rad/s)
constexpr float CALI_REVERSE_RADPS = -3.0f;

/// 校准正转目标角度 (rad)
constexpr float CALI_FORWARD_ANGLE = 0.0f;

/// 堵转检测速度误差阈值 (70%)
constexpr float CALI_BLOCK_THRESHOLD = 0.7f;

/// 堵转检测时间阈值 (ms)
constexpr uint16_t CALI_BLOCK_TIME_MS = 1500;

/// 校准完成角度误差阈值 (rad)
constexpr float CALI_DONE_ANGLE_THRESHOLD = 0.01f;

/// 单发完成角度误差阈值 (rad)
constexpr float SINGLE_DONE_ANGLE_THRESHOLD = 0.003f;

/// 单发超时 (ms)
constexpr uint16_t SINGLE_DONE_TIMEOUT_MS = 500;

} // namespace pyro

#endif // __LARGE_LAUNCHER_CONFIG_H__
