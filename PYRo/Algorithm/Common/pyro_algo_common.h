#ifndef PYRO_ALGO_COMMON_H
#define PYRO_ALGO_COMMON_H

#include "pyro_core_def.h"
#include <cmath>
#include <cstdint>
#include <optional>

namespace pyro
{
// ================== 基础数学运算 ==================
float wrap2pi_f32(float input);
float radps_to_rpm(float radps);
float calculate_angle_diff(float current, float target);
float evaluate_polynomial(float x, const float *coeffs, uint32_t degree);
float mps_to_rpm(float mps, float radius);
float rpm_to_mps(const float rpm, const float radius);
float loop_fp32_constrain(float val, float min_val, float max_val);

// ================== 弹道解算接口 ==================
/**
 * @brief 带有二次空气阻力的 Pitch 角弹道求解器
 * @param delta_x 目标相对云台的 X 轴坐标差 (米)
 * @param delta_y 目标相对云台的 Y 轴坐标差 (米)
 * @param delta_z 目标相对云台的高度差 (米)
 * @param v0 弹丸初速度 (m/s)
 * @param pitch_guess 初始迭代猜测角 (rad)，默认使用 0.2 弧度
 * @return std::optional<double> 求解成功则返回理想 Pitch 弧度，失败则返回 std::nullopt
 */
std::optional<double> solveIdealPitch(
    double delta_x,
    double delta_y,
    double delta_z,
    double v0,
    double pitch_guess = 0.2
);

} // namespace pyro
#endif