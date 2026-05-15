#include "pyro_algo_common.h"

namespace pyro
{
    // ---------------- 原有基础算法保留 ----------------
    float wrap2pi_f32(float input) {
        return fmodf(input, 2 * PI);
    }

    float radps_to_rpm(const float radps) {
        return radps * 9.5492966f;
    }

    float calculate_angle_diff(float current, float target) {
        const float diff = std::fabs(current - target);
        return std::fmin(diff, 2 * PI - diff);
    }

    float evaluate_polynomial(const float x, const float *coeffs, const uint32_t degree) {
        float result = coeffs[0];
        for (uint32_t i = 1; i <= degree; ++i) {
            result = result * x + coeffs[i];
        }
        return result;
    }

    float mps_to_rpm(const float mps, const float radius) {
        if (radius < 1e-4f) return 0.0f;
        return (mps / radius) * 9.5492966f;
    }

    float rpm_to_mps(const float rpm, const float radius) {
        return rpm * radius / 9.5492966f;
    }

    float loop_fp32_constrain(float val, const float min_val, const float max_val) {
        const float len = max_val - min_val;
        if (len < 1e-6f) return val;
        while (val > max_val) val -= len;
        while (val < min_val) val += len;
        return val;
    }

    // ---------------- 弹道积分模型与牛顿法构建 ----------------
    namespace { 
        inline double f_func(double p) {
            double sq = std::sqrt(1.0 + p * p);
            return p * sq + std::log(p + sq);
        }

        double calcIntegralX(double p0, double p1, double c, double k) {
            constexpr int N = 60;
            double dp = (p0 - p1) / N;
            double sum = 0.0;
            for (int i = 0; i <= N; ++i) {
                double p = p1 + i * dp;
                double denominator = c - f_func(p);
                if (denominator <= 1e-6) denominator = 1e-6;
                double val = 1.0 / denominator;
                double weight = (i == 0 || i == N) ? 1.0 : ((i % 2 == 1) ? 4.0 : 2.0);
                sum += weight * val;
            }
            return (dp / 3.0) * sum / k;
        }

        double calcIntegralY(double p0, double p1, double c, double k) {
            constexpr int N = 60;
            double dp = (p0 - p1) / N;
            double sum = 0.0;
            for (int i = 0; i <= N; ++i) {
                double p = p1 + i * dp;
                double denominator = c - f_func(p);
                if (denominator <= 1e-6) denominator = 1e-6;
                double val = p / denominator;
                double weight = (i == 0 || i == N) ? 1.0 : ((i % 2 == 1) ? 4.0 : 2.0);
                sum += weight * val;
            }
            return (dp / 3.0) * sum / k;
        }

        // [核心修改点]：剔除 Eigen，使用二维数组和代数公式直接求逆
        bool calcJacobianInv(double p0, double p1, double v0, double k, double g, double x0, double y0, double J_inv[2][2]) {
            constexpr double eps = 1e-5;

            // 1. 基准值计算
            double c_base = g * (1.0 + p0 * p0) / (k * v0 * v0) + f_func(p0);
            double D0_base = x0 - calcIntegralX(p0, p1, c_base, k);
            double D1_base = y0 - calcIntegralY(p0, p1, c_base, k);

            // 2. 扰动 p1 求偏导
            double p1_eps = p1 + eps;
            double x_p1 = calcIntegralX(p0, p1_eps, c_base, k);
            double y_p1 = calcIntegralY(p0, p1_eps, c_base, k);
            double dD0_dp1 = (x0 - x_p1 - D0_base) / eps;
            double dD1_dp1 = (y0 - y_p1 - D1_base) / eps;

            // 3. 扰动 p0 求偏导
            double p0_eps = p0 + eps;
            double c_eps = g * (1.0 + p0_eps * p0_eps) / (k * v0 * v0) + f_func(p0_eps);
            double x_p0 = calcIntegralX(p0_eps, p1, c_eps, k);
            double y_p0 = calcIntegralY(p0_eps, p1, c_eps, k);
            double dD0_dp0 = (x0 - x_p0 - D0_base) / eps;
            double dD1_dp0 = (y0 - y_p0 - D1_base) / eps;

            // 4. 构建 2x2 雅可比矩阵：
            // J = [ a, b ]
            //     [ c, d ]
            double a = dD0_dp1;
            double b = dD0_dp0;
            double c = dD1_dp1;
            double d = dD1_dp0;

            // 5. 使用代数公式计算 2x2 矩阵求逆（比 DSP 库更快）
            double det = a * d - b * c;
            if (std::abs(det) < 1e-6) {
                return false; // 矩阵奇异，求解失败
            }

            double inv_det = 1.0 / det;
            J_inv[0][0] = d * inv_det;
            J_inv[0][1] = -b * inv_det;
            J_inv[1][0] = -c * inv_det;
            J_inv[1][1] = a * inv_det;

            return true;
        }
    }

    // ---------------- 主干解算函数 ----------------
    std::optional<double> solveIdealPitch(
        double delta_x, double delta_y, double delta_z, double v0, double pitch_guess)
    {
        constexpr double k = 0.01903;
        constexpr double g = 9.81;
        constexpr int max_iter = 30;
        constexpr double tolerance = 1e-5;

        double x0 = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double y0 = delta_z;

        double v_x_approx = v0 * std::cos(pitch_guess);
        if (v_x_approx < 1e-3) return std::nullopt;

        double t_approx = x0 / v_x_approx;

        // 状态变量提取为简单的 double 类型
        double p0 = std::tan(pitch_guess);
        double p1 = (v0 * std::sin(pitch_guess) - g * t_approx) / v_x_approx;

        // [核心修改点]：替换 Eigen 向量和矩阵
        double D0, D1;
        double J_inv[2][2];

        for (int i = 0; i < max_iter; ++i) {
            double c = g * (1.0 + p0 * p0) / (k * v0 * v0) + f_func(p0);

            D0 = x0 - calcIntegralX(p0, p1, c, k);
            D1 = y0 - calcIntegralY(p0, p1, c, k);

            // 替换 Eigen 的 D.norm()
            if (std::sqrt(D0 * D0 + D1 * D1) < tolerance) {
                return std::atan(p0);
            }

            if (!calcJacobianInv(p0, p1, v0, k, g, x0, y0, J_inv)) {
                break;
            }

            // [核心修改点]：展开矩阵乘法 R += J_inv * D
            double dp1 = J_inv[0][0] * D0 + J_inv[0][1] * D1;
            double dp0 = J_inv[1][0] * D0 + J_inv[1][1] * D1;

            p1 += dp1;
            p0 += dp0;
        }

        return std::nullopt;
    }

} // namespace pyro