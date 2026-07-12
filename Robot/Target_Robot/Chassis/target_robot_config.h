#ifndef __TARGET_ROBOT_CONFIG_H__
#define __TARGET_ROBOT_CONFIG_H__

#include <cstdint>

// =========================================================
// Target Robot 底盘物理参数
// =========================================================

/// 全向轮半径 (m) — 对应原工程 omnid 参数
constexpr float TARGET_OMNI_WHEEL_RADIUS = 0.12f;

/// 舵轮半径 (m) — 对应原工程 rudd 参数
constexpr float TARGET_RUD_WHEEL_RADIUS  = 0.12f;

/// 全向轮到 y 轴距离 (m)
constexpr float TARGET_OMNI_DTY = 0.183f;

/// 全向轮到 x 轴距离 (m)
constexpr float TARGET_OMNI_DTX = 0.168f;

/// 舵轮到 y 轴距离 (m)
constexpr float TARGET_RUD_DTY  = 0.178f;

/// 舵轮到 x 轴距离 (m)
constexpr float TARGET_RUD_DTX  = 0.164f;

#endif // __TARGET_ROBOT_CONFIG_H__
