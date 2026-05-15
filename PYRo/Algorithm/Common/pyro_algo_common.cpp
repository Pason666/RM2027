#include "pyro_algo_common.h"

#include "arm_math.h"
#include <cmath>

namespace pyro
{

float wrap2pi_f32(const float input)
{
    return std::fmod(input, 2.0f * PI);
}

float radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

float calculate_angle_diff(const float current, const float target)
{
    const float diff = std::fabs(current - target);
    return std::fmin(diff, 2.0f * PI - diff);
}

float evaluate_polynomial(const float x, const float *coeffs,
                          const uint32_t degree)
{
    float result = coeffs[0];
    for (uint32_t i = 1; i <= degree; ++i)
    {
        result = result * x + coeffs[i];
    }
    return result;
}

float mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1.0e-4f)
    {
        return 0.0f;
    }
    return (mps / radius) * 9.5492966f;
}

float rpm_to_mps(const float rpm, const float radius)
{
    return rpm * radius / 9.5492966f;
}

float loop_fp32_constrain(float val, const float min_val, const float max_val)
{
    const float len = max_val - min_val;
    if (len < 1.0e-6f)
    {
        return val;
    }
    while (val > max_val)
    {
        val -= len;
    }
    while (val < min_val)
    {
        val += len;
    }
    return val;
}

namespace
{
constexpr float kDrag    = 0.0112f; // 二次空气阻力系数，需随弹丸和场地标定
constexpr float kGravity = 9.81f;   // 重力加速度，单位 m/s^2
constexpr float kDenominatorMin = 1.0e-6f; // 积分分母下限，避免接近奇点时除零
constexpr float kJacobianStep   = 1.0e-4f; // 有限差分步长，过小易受浮点噪声影响
constexpr float kSingularEpsilon =
    1.0e-6f; // 雅可比行列式阈值，小于该值视为不可逆
constexpr float kSolveTolerance = 1.0e-3f; // 残差收敛阈值，单位约等于米
constexpr int kIntegralSteps    = 60;      // Simpson 积分分段数，需为偶数
constexpr int kMaxIterations    = 30; // Newton 最大迭代次数，防止异常输入卡死

float safeSqrt(const float value)
{
    float result = 0.0f;
    // CMSIS-DSP sqrt keeps this path in f32 and avoids accidental double math.
    arm_sqrt_f32(value > 0.0f ? value : 0.0f, &result);
    return result;
}

float trajectoryFunc(const float p)
{
    // p is dy/dx, i.e. tan(theta). This is the primitive function that appears
    // after rewriting the drag model with slope as the integration variable.
    const float sq = safeSqrt(1.0f + p * p);
    return p * sq + std::log(p + sq);
}

float calcIntegralX(const float p0, const float p1, const float c,
                    const float k)
{
    // Simpson integration from terminal slope p1 to initial slope p0. The
    // result is the horizontal displacement predicted by the current state.
    const float dp = (p0 - p1) / static_cast<float>(kIntegralSteps);
    float sum      = 0.0f;

    for (int i = 0; i <= kIntegralSteps; ++i)
    {
        const float p     = p1 + static_cast<float>(i) * dp;
        float denominator = c - trajectoryFunc(p);
        if (denominator < kDenominatorMin)
        {
            denominator = kDenominatorMin;
        }

        const float weight = (i == 0 || i == kIntegralSteps) ? 1.0f
                             : ((i & 1) != 0)                ? 4.0f
                                                             : 2.0f;
        sum += weight / denominator;
    }

    return (dp / 3.0f) * sum / k;
}

float calcIntegralY(const float p0, const float p1, const float c,
                    const float k)
{
    // Same integration interval as X, with an extra slope multiplier to obtain
    // vertical displacement.
    const float dp = (p0 - p1) / static_cast<float>(kIntegralSteps);
    float sum      = 0.0f;

    for (int i = 0; i <= kIntegralSteps; ++i)
    {
        const float p     = p1 + static_cast<float>(i) * dp;
        float denominator = c - trajectoryFunc(p);
        if (denominator < kDenominatorMin)
        {
            denominator = kDenominatorMin;
        }

        const float weight = (i == 0 || i == kIntegralSteps) ? 1.0f
                             : ((i & 1) != 0)                ? 4.0f
                                                             : 2.0f;
        sum += weight * p / denominator;
    }

    return (dp / 3.0f) * sum / k;
}

float calcIntegralC(const float p0, const float v0)
{
    // c is the conserved term derived from initial slope and muzzle speed.
    return kGravity * (1.0f + p0 * p0) / (kDrag * v0 * v0) + trajectoryFunc(p0);
}

bool calcJacobianInv(const float p0, const float p1, const float v0,
                     const float x0, const float y0, float (&j_inv_data)[4])
{
    // Residual D = target displacement - predicted displacement.
    // The Newton state is [p1, p0]^T, so columns are finite differences with
    // respect to terminal slope p1 and initial slope p0.
    const float c_base  = calcIntegralC(p0, v0);
    const float d0_base = x0 - calcIntegralX(p0, p1, c_base, kDrag);
    const float d1_base = y0 - calcIntegralY(p0, p1, c_base, kDrag);

    // First column: perturb terminal slope p1 while c stays unchanged.
    const float p1_eps  = p1 + kJacobianStep;
    const float x_p1    = calcIntegralX(p0, p1_eps, c_base, kDrag);
    const float y_p1    = calcIntegralY(p0, p1_eps, c_base, kDrag);
    const float dD0_dp1 = (x0 - x_p1 - d0_base) / kJacobianStep;
    const float dD1_dp1 = (y0 - y_p1 - d1_base) / kJacobianStep;

    // Second column: perturb initial slope p0, which also changes c.
    const float p0_eps  = p0 + kJacobianStep;
    const float c_eps   = calcIntegralC(p0_eps, v0);
    const float x_p0    = calcIntegralX(p0_eps, p1, c_eps, kDrag);
    const float y_p0    = calcIntegralY(p0_eps, p1, c_eps, kDrag);
    const float dD0_dp0 = (x0 - x_p0 - d0_base) / kJacobianStep;
    const float dD1_dp0 = (y0 - y_p0 - d1_base) / kJacobianStep;

    float j_data[4]     = {
        dD0_dp1,
        dD0_dp0,
        dD1_dp1,
        dD1_dp0,
    };

    const float det = dD0_dp1 * dD1_dp0 - dD0_dp0 * dD1_dp1;
    if (std::fabs(det) < kSingularEpsilon)
    {
        return false;
    }

    arm_matrix_instance_f32 j{};
    arm_matrix_instance_f32 j_inv{};
    arm_mat_init_f32(&j, 2, 2, j_data);
    arm_mat_init_f32(&j_inv, 2, 2, j_inv_data);

    // Use CMSIS-DSP for the matrix inverse so this remains consistent with the
    // rest of the control and filter code on Cortex-M.
    return arm_mat_inverse_f32(&j, &j_inv) == ARM_MATH_SUCCESS;
}

bool calcNewtonStep(float (&j_inv_data)[4], const float d0, const float d1,
                    float &dp1, float &dp0)
{
    // step = inv(J) * D. The caller adds this step to [p1, p0].
    float d_data[2]    = {d0, d1};
    float step_data[2] = {};

    arm_matrix_instance_f32 j_inv{};
    arm_matrix_instance_f32 d{};
    arm_matrix_instance_f32 step{};
    arm_mat_init_f32(&j_inv, 2, 2, j_inv_data);
    arm_mat_init_f32(&d, 2, 1, d_data);
    arm_mat_init_f32(&step, 2, 1, step_data);

    if (arm_mat_mult_f32(&j_inv, &d, &step) != ARM_MATH_SUCCESS)
    {
        return false;
    }

    dp1 = step_data[0];
    dp0 = step_data[1];
    return true;
}
} // namespace

std::optional<float> solveParabolicPitch(const float delta_x,
                                         const float delta_y,
                                         const float delta_z, const float v0,
                                         const bool use_high_root)
{
    if (v0 < 1.0e-3f)
    {
        return std::nullopt;
    }

    const float x = safeSqrt(delta_x * delta_x + delta_y * delta_y);
    if (x < 1.0e-4f)
    {
        return std::nullopt;
    }

    // y = x*tan(theta) - g*x^2/(2*v0^2) * (1 + tan(theta)^2)
    // Let u = tan(theta), a = g*x^2/(2*v0^2):
    // a*u^2 - x*u + (a + y) = 0.
    const float v0_sq = v0 * v0;
    const float a     = kGravity * x * x / (2.0f * v0_sq);
    const float disc  = x * x - 4.0f * a * (a + delta_z);
    if (disc < 0.0f || a < 1.0e-6f)
    {
        return std::nullopt;
    }

    const float sqrt_disc = safeSqrt(disc);
    const float tan_pitch = use_high_root ? ((x + sqrt_disc) / (2.0f * a))
                                          : ((x - sqrt_disc) / (2.0f * a));

    if (!std::isfinite(tan_pitch))
    {
        return std::nullopt;
    }

    return std::atan(tan_pitch);
}

std::optional<float> solveIdealPitch(const float delta_x, const float delta_y,
                                     const float delta_z, const float v0,
                                     std::optional<float> pitch_guess)
{
    if (v0 < 1.0e-3f)
    {
        return std::nullopt;
    }

    if (!pitch_guess.has_value())
    {
        pitch_guess = solveParabolicPitch(delta_x, delta_y, delta_z, v0, false);
        if (!pitch_guess.has_value())
        {
            return std::nullopt;
        }
    }

    // Convert the 3D target offset into the 2D ballistic plane.
    const float x0         = safeSqrt(delta_x * delta_x + delta_y * delta_y);
    const float y0         = delta_z;

    // A near-vertical initial guess makes the horizontal time estimate
    // unstable.
    const float v_x_approx = v0 * std::cos(*pitch_guess);
    if (v_x_approx < 1.0e-3f)
    {
        return std::nullopt;
    }

    // Initial slopes: p0 is muzzle slope, p1 is an ideal no-drag terminal slope
    // used only to seed Newton iteration.
    const float t_approx = x0 / v_x_approx;
    float p0             = std::tan(*pitch_guess);
    float p1 = (v0 * std::sin(*pitch_guess) - kGravity * t_approx) / v_x_approx;

    for (int i = 0; i < kMaxIterations; ++i)
    {
        // D = target - prediction. Once D is small enough, atan(p0) is the
        // solved pitch angle.
        const float c        = calcIntegralC(p0, v0);
        const float d0       = x0 - calcIntegralX(p0, p1, c, kDrag);
        const float d1       = y0 - calcIntegralY(p0, p1, c, kDrag);
        const float residual = safeSqrt(d0 * d0 + d1 * d1);

        if (residual < kSolveTolerance)
        {
            return std::atan(p0);
        }

        // Solve the local linear system using DSP matrix primitives.
        float j_inv_data[4] = {};
        if (!calcJacobianInv(p0, p1, v0, x0, y0, j_inv_data))
        {
            return std::nullopt;
        }

        float dp1 = 0.0f;
        float dp0 = 0.0f;
        if (!calcNewtonStep(j_inv_data, d0, d1, dp1, dp0))
        {
            return std::nullopt;
        }

        p1 += dp1;
        p0 += dp0;

        if (!std::isfinite(p0) || !std::isfinite(p1))
        {
            return std::nullopt;
        }
    }

    return std::nullopt;
}

} // namespace pyro
