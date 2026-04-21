#include "pyro_power_control.h"

namespace pyro
{

power_controller_t &power_controller_t::get_instance()
{
    static power_controller_t instance;
    return instance;
}

power_controller_t::power_controller_t()
    : _registered_count(0), _safe_energy_ref(40.0f),
      _buffer_pid(0.5f, 0.0f, 0.0f, 0.0f, 20.0f), _last_total_predict(0.0f)
{
}

power_node_t *
power_controller_t::register_motor(const power_fit_params_t &params)
{
    if (_registered_count >= MAX_MOTORS)
    {
        return nullptr;
    }
    power_node_t *node = &_nodes[_registered_count++];
    node->params       = params;
    node->is_active    = true;
    return node;
}

void power_controller_t::config_buffer_pid(const float safe_energy,
                                           const float kp, const float ki,
                                           const float kd)
{
    _safe_energy_ref = safe_energy;
    _buffer_pid.set_gains(kp, ki, kd);
}

void power_controller_t::solve(const float referee_power_limit,
                               const float current_buffer_energy,
                               const float cap_extra_power)
{
    // 1. [裁判系统端] 仅根据缓冲能量计算基础的功率补偿
    float p_adjust =
        _buffer_pid.calculate(_safe_energy_ref, current_buffer_energy);

    // 极端危险状态的强力惩罚机制，强行从上限中抠出功率回血
    if (current_buffer_energy < DANGER_ENERGY_THRESH)
    {
        p_adjust -= (DANGER_ENERGY_THRESH - current_buffer_energy) *
                    DANGER_PENALTY_FACTOR;
    }

    // 纯裁判系统视角的动态功率限制
    float referee_dyn_limit = referee_power_limit + p_adjust;

    // 如果缓冲能量见底，绝对不允许消耗【裁判系统】的功率
    if (current_buffer_energy < DEAD_ENERGY_THRESH)
    {
        referee_dyn_limit = 0.0f;
    }
    else
    {
        // 限制裁判系统端不能输出负功率
        referee_dyn_limit = std::max(referee_dyn_limit, 0.0f);
    }

    // [核心解耦] 最终总动态功率限制 = 裁判系统允许功率 + 电容端提供的额外功率
    float dyn_limit = referee_dyn_limit + cap_extra_power;
    dyn_limit = std::max(dyn_limit, 0.0f); // 兜底保护，总功率不为负

    // 2. 遍历内部节点，计算预测功率与二次方程系数
    float A = 0.0f, B = 0.0f, C = 0.0f;
    _last_total_predict = 0.0f;

    for (size_t i = 0; i < _registered_count; ++i)
    {
        power_node_t *m = &_nodes[i];
        if (!m->is_active)
            continue;

        // 温度修正系数 (限制下限防传感器断连)
        const float temp_factor = std::max(
            1.0f + m->params.alpha * (m->temp - TEMP_OFFSET), MIN_TEMP_FACTOR);

        // 二次项 A (铜损)
        const float a_i =
            m->params.k2 * temp_factor * m->target_cmd * m->target_cmd;
        A += a_i;

        // 一次项 B (机械功，不考虑动能回收)
        const float p_mech = m->params.k1 * m->target_cmd * m->rpm;
        const float b_i    = (p_mech > 0.0f) ? p_mech : 0.0f;
        B += b_i;

        // 常数项 C (铁耗、粘滞与库仑摩擦、静态功耗)
        const float c_i = m->params.k3 * m->rpm * m->rpm +
                          m->params.k4 * std::abs(m->rpm) + m->params.k5;
        C += c_i;

        m->power_predict = a_i + b_i + c_i;
        _last_total_predict += m->power_predict;
    }

    // 3. 求解降额缩放系数 K
    float k = 1.0f;
    if (_last_total_predict > dyn_limit)
    {
        const float c_term = C - dyn_limit;
        if (c_term > 0.0f)
        {
            k = 0.0f; // 仅常数耗电就已超标，完全切断扭矩
        }
        else
        {
            if (A > 1e-6f)
            {
                const float delta = B * B - 4.0f * A * c_term;
                k = (delta >= 0.0f) ? ((-B + std::sqrt(delta)) / (2.0f * A))
                                    : 0.0f;
            }
            else if (B > 1e-6f)
            {
                k = -c_term / B;
            }
        }
        k = std::clamp(k, 0.0f, 1.0f);
    }

    // 4. 更新安全指令并作一阶低通滤波防抖
    for (size_t i = 0; i < _registered_count; ++i)
    {
        power_node_t *m = &_nodes[i];
        if (!m->is_active)
            continue;

        const float target_cmd = m->target_cmd * k;
        m->safe_cmd =
            FILTER_ALPHA * target_cmd + (1.0f - FILTER_ALPHA) * m->last_cmd;
        m->last_cmd = m->safe_cmd;
    }
}

} // namespace pyro