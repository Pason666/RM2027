/**
 * @file: pyro_power_control.h
 * @brief: 功率控制驱动头文件（终极灰盒拟合版 + 代理节点架构）
 */
#ifndef __PYRO_POWER_CONTROL_H__
#define __PYRO_POWER_CONTROL_H__

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "pyro_algo_pid.h" // 引入您现有的 PID 算法库

namespace pyro
{

/**
 * @brief 灰盒拟合参数 (由 MATLAB cftool 或 lsqcurvefit 跑出)
 */
struct power_fit_params_t
{
    float k1{0.0f};        ///< 机械功率系数 (对应 cmd * rpm)
    float k2{0.0f};        ///< 铜损/热损耗系数 (对应 cmd^2)
    float k3{0.0f};        ///< 高频损耗系数，如涡流/粘滞摩擦 (对应 rpm^2)
    float k4{0.0f};        ///< 低频损耗系数，如磁滞/库仑摩擦 (对应 |rpm|)
    float k5{1.0f};        ///< 静态基础功耗
    float alpha{0.00393f}; ///< 电阻温度系数 (铜通常为 0.00393)
    // 采用 C++11 默认成员初始化，无需显式写构造函数
};

/**
 * @brief 暴露给外部的功率节点代理
 */
struct power_node_t
{
    // --- Input ---
    float target_cmd{0.0f}; ///< 期望下发的原始指令 (如 -16384 ~ 16384)
    float rpm{0.0f};        ///< 当前电机反馈转速 (如 RPM)
    float temp{20.0f};      ///< 当前电机温度 (℃)

    // --- Output ---
    float safe_cmd{0.0f};      ///< 限制后的安全指令
    float power_predict{0.0f}; ///< 当前节点预测耗电功率 (W)

    // --- Internal ---
    float last_cmd{0.0f};
    power_fit_params_t params{};
    bool is_active{false};
};

/**
 * @brief 全局功率控制器 (单例模式)
 */
class power_controller_t
{
  public:
    static power_controller_t &get_instance();

    // 禁用拷贝和赋值，确保严格单例
    power_controller_t(const power_controller_t &)            = delete;
    power_controller_t &operator=(const power_controller_t &) = delete;

    power_node_t *register_motor(const power_fit_params_t &params);
    void config_buffer_pid(float safe_energy, float kp, float ki, float kd);
    void solve(float referee_power_limit, float current_buffer_energy);

    [[nodiscard]] float get_total_predicted_power() const
    {
        return _last_total_predict;
    }

  private:
    power_controller_t();
    ~power_controller_t()                        = default;

    // ================= 编译期常量集中管理区 =================
    static constexpr size_t MAX_MOTORS           = 8; ///< 最大支持节点数

    // --- 算法核心系数 ---
    ///< 安全指令一阶低通滤波系数
    static constexpr float FILTER_ALPHA          = 0.85f;
    ///< 电机标称/默认温度 (℃)
    static constexpr float DEFAULT_TEMP          = 20.0f;
    ///< 最小温度修正系数(防传感器断连)
    static constexpr float MIN_TEMP_FACTOR       = 0.1f;


    // --- 缓冲能量与功率动态调节阈值 ---
    ///< 触发强力惩罚机制的能量阈值 (J)
    static constexpr float DANGER_ENERGY_THRESH  = 15.0f;
    ///< 强制切断功率输出的能量死区阈值 (J)
    static constexpr float DEAD_ENERGY_THRESH    = 5.0f;
    ///< 危险状态回血惩罚倍率
    static constexpr float DANGER_PENALTY_FACTOR = 5.0f;
    ///< 动态功率超发倍数限制
    static constexpr float DYN_LIMIT_MULTIPLIER  = 1.5f;
    ///< 动态功率超发绝对容差 (W)
    static constexpr float DYN_LIMIT_OFFSET      = 40.0f;

    // ========================================================

    power_node_t _nodes[MAX_MOTORS];
    size_t _registered_count;

    float _safe_energy_ref;
    pyro::pid_t _buffer_pid;
    float _last_total_predict;
};

} // namespace pyro

#endif // __PYRO_POWER_CONTROL_H__