#ifndef PYRO_ALGO_COMMON_H
#define PYRO_ALGO_COMMON_H

#include "pyro_core_def.h"
#include <cstdint>
#include <optional>

namespace pyro
{

float wrap2pi_f32(float input);
float radps_to_rpm(float radps);
float calculate_angle_diff(float current, float target);
float evaluate_polynomial(float x, const float *coeffs, uint32_t degree);
float mps_to_rpm(float mps, float radius);
float rpm_to_mps(float rpm, float radius);
float loop_fp32_constrain(float val, float min_val, float max_val);

/**
 * @brief Solve the ideal no-drag parabolic pitch angle.
 *
 * The solution comes from substituting t = x / (v0 * cos(theta)) into
 * y = v0 * sin(theta) * t - 0.5f * g * t^2, which produces a quadratic
 * equation in tan(theta). The low root is the flatter trajectory; the high root
 * is the lob trajectory.
 *
 * @param delta_x Target X offset in meters.
 * @param delta_y Target Y offset in meters.
 * @param delta_z Target height offset in meters.
 * @param v0 Muzzle speed in m/s.
 * @param use_high_root true for the high-angle root, false for the low root.
 * @return Pitch angle in radians when the target is reachable.
 */
std::optional<float> solveParabolicPitch(float delta_x, float delta_y,
                                         float delta_z, float v0,
                                         bool use_high_root);

/**
 * @brief Solve the pitch angle for a projectile with quadratic air drag.
 *
 * The target position is expressed in the gimbal frame. delta_x and delta_y are
 * collapsed into horizontal distance internally, while delta_z is the vertical
 * height difference. The solver returns std::nullopt when the Newton iteration
 * fails to converge or the Jacobian becomes singular.
 *
 * If pitch_guess is not provided, the low-root no-drag parabolic solution is
 * used as the initial guess.
 *
 * @param delta_x Target X offset in meters.
 * @param delta_y Target Y offset in meters.
 * @param delta_z Target height offset in meters.
 * @param v0 Muzzle speed in m/s.
 * @param pitch_guess Optional initial pitch guess in radians.
 * @return Pitch angle in radians when solved.
 */
std::optional<float>
solveIdealPitch(float delta_x, float delta_y, float delta_z, float v0,
                std::optional<float> pitch_guess = std::nullopt);

} // namespace pyro

#endif // PYRO_ALGO_COMMON_H
