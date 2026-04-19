#include "pyro_power_control_drv.h"

namespace pyro
{

PowerController& PowerController::getInstance(size_t motor_num)
{
    static PowerController instance(motor_num);
    return instance;
}

PowerController::PowerController(size_t motor_num)
    : _params(motor_num), _integral_err(0.0f), _last_err(0.0f), _last_total_predict(0.0f)
{
}

void PowerController::setMotorParams(size_t motor_index, const FitParams& params)
{
    if (motor_index < _params.size()) {
        _params[motor_index] = params;
    }
}

void PowerController::setBufferPid(const BufferPidParams& pid_params)
{
    _pid_params = pid_params;
    _integral_err = 0.0f; // 重置积分器
    _last_err = 0.0f;
}

float PowerController::updateDynamicLimit(float referee_limit, float buffer_energy)
{
    float error = buffer_energy - _pid_params.safe_energy;

    _integral_err += error;
    _integral_err = std::clamp(_integral_err, -50.0f, 50.0f); // 积分抗饱和

    float derivative = error - _last_err;
    _last_err = error;

    float p_adjust = _pid_params.kp * error +
                     _pid_params.ki * _integral_err +
                     _pid_params.kd * derivative;

    // 极端危险状态的强力惩罚机制
    if (buffer_energy < 15.0f) {
        p_adjust -= (15.0f - buffer_energy) * 5.0f;
    }

    float dyn_limit = referee_limit + p_adjust;

    // 限制范围: 不能小于0，最大允许一定程度超发(吃电容/飞轮)
    return std::clamp(dyn_limit, 0.0f, referee_limit * 1.5f + 30.0f);
}

void PowerController::solve(std::vector<MotorData>& motors,
                            float referee_power_limit,
                            float current_buffer_energy,
                            const std::vector<float>* power_ratios)
{
    float dyn_limit = updateDynamicLimit(referee_power_limit, current_buffer_energy);

    // 强制断电保护：避免掉血
    if (current_buffer_energy < 5.0f) {
        dyn_limit = 0.0f;
    }

    float A = 0.0f, B = 0.0f, C = 0.0f;
    _last_total_predict = 0.0f;

    size_t num = std::min(motors.size(), _params.size());

    // 1. 遍历计算二次方程系数
    for (size_t i = 0; i < num; ++i) {
        const auto& p = _params[i];
        const auto& m = motors[i];

        // 温度修正系数
        float temp_factor = 1.0f + p.alpha * (m.temp - 20.0f);
        // 如果温度传感器离线返回了极低温度，防止出现负电阻
        if (temp_factor < 0.1f) temp_factor = 1.0f;

        // 二次项 A (对应 cmd^2，纯铜损)
        float a_i = p.k2 * temp_factor * m.cmd * m.cmd;
        A += a_i;

        // 一次项 B (对应 cmd * rpm，做机械功)
        float p_mech = p.k1 * m.cmd * m.rpm;
        float b_i = (p_mech > 0.0f) ? p_mech : 0.0f; // 不回收动能
        B += b_i;

        // 常数项 C (对应 rpm^2, |rpm|, 静态常数)
        float c_i = p.k3 * m.rpm * m.rpm + p.k4 * std::abs(m.rpm) + p.k5;
        C += c_i;

        motors[i].power_predict = a_i + b_i + c_i;
        _last_total_predict += motors[i].power_predict;
    }

    // 2. 求解最佳功率缩放系数 k (A*k^2 + B*k + C - dyn_limit <= 0)
    float k = 1.0f;

    if (_last_total_predict > dyn_limit) {
        float c_term = C - dyn_limit;

        if (c_term > 0) {
            k = 0.0f; // 仅摩擦和静态耗电已超过限制，必须完全切断动力输出
        } else {
            if (A > 1e-6f) {
                float delta = B * B - 4 * A * c_term;
                k = (delta >= 0) ? ((-B + std::sqrt(delta)) / (2 * A)) : 0.0f;
            } else if (B > 1e-6f) {
                k = -c_term / B;
            }
        }
        k = std::clamp(k, 0.0f, 1.0f);
    }

    // 3. 应用限幅与一阶低通滤波，输出最终安全指令
    const float FILTER_ALPHA = 0.85f;

    for (size_t i = 0; i < num; ++i) {
        float target_cmd = motors[i].cmd * k;

        // 可选：独立比例限制逻辑 (如果传入了 power_ratios, 则覆盖上述统一 k)
        // ... 如果你的步兵/英雄需要极高精度的麦轮受力分配，可在此处展开针对每个轮子解方程 ...

        motors[i].safe_cmd = FILTER_ALPHA * target_cmd +
                            (1.0f - FILTER_ALPHA) * motors[i].last_cmd;

        motors[i].last_cmd = motors[i].safe_cmd;
    }
}

} // namespace pyro