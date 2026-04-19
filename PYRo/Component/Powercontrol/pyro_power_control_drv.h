/**
 * @file: pyro_power_control.h
 * @brief: 功率控制驱动头文件（终极灰盒拟合版）
 * @note: 本模块参数无需关注物理单位，k1~k5 通过 MATLAB 直接对原始下发指令与反馈转速拟合得出。
 */
#ifndef __PYRO__POWER_CONTROL_H__
#define __PYRO__POWER_CONTROL_H__

#include <vector>
#include <cmath>
#include <algorithm>

namespace pyro
{

class PowerController
{
public:
    /**
     * @brief 灰盒拟合参数 (由 MATLAB cftool 或 lsqcurvefit 跑出)
     * 模型: P = max(0, k1*cmd*w) + k2*(1+alpha*dT)*cmd^2 + k3*w^2 + k4*|w| + k5
     */
    struct FitParams {
        float k1;    ///< 机械功率系数 (对应 cmd * w)
        float k2;    ///< 铜损/热损耗系数 (对应 cmd^2)
        float k3;    ///< 高频损耗系数，如涡流/粘滞摩擦 (对应 w^2)
        float k4;    ///< 低频损耗系数，如磁滞/库仑摩擦 (对应 |w|)
        float k5;    ///< 静态基础功耗 (零输入零转速时的裁判系统功率)
        float alpha; ///< 电阻温度系数 (铜通常为 0.00393)

        FitParams() : k1(0), k2(0), k3(0), k4(0), k5(1.0f), alpha(0.00393f) {}
    };

    /**
     * @brief 缓冲能量 PID 闭环参数
     */
    struct BufferPidParams {
        float kp, ki, kd;
        float safe_energy;  ///< 期望维持的安全缓冲能量 (如 40J)

        BufferPidParams() : kp(1.5f), ki(0.01f), kd(0.1f), safe_energy(40.0f) {}
    };

    /**
     * @brief 单个电机的数据交互结构体
     */
    struct MotorData {
        // --- 输入区 (每次计算前更新) ---
        float cmd;           ///< 拟发送的原始指令 (如 -16384 ~ 16384)
        float rpm;           ///< 原始反馈转速 (如 RPM)
        float temp;          ///< 当前电机温度 (℃)

        // --- 内部状态区 ---
        float last_cmd;      ///< 内部维护的上一拍指令(用于低通滤波)

        // --- 输出区 (计算后读取) ---
        float safe_cmd;      ///< 经过功率限制后的安全发送指令
        float power_predict; ///< 当前电机预测耗电功率 (W)

        MotorData() : cmd(0), rpm(0), temp(20.0f), last_cmd(0), safe_cmd(0), power_predict(0) {}
    };

    /**
     * @brief 获取单例实例 (假设底盘默认 4 个电机)
     */
    static PowerController& getInstance(size_t motor_num = 4);

    // --- 配置接口 ---
    void setMotorParams(size_t motor_index, const FitParams& params);
    void setBufferPid(const BufferPidParams& pid_params);

    // --- 核心计算接口 ---
    /**
     * @brief 执行功率限制计算
     * @param motors 电机数据数组引用
     * @param referee_power_limit 裁判系统允许的最大功率
     * @param current_buffer_energy 裁判系统反馈的当前缓冲能量
     * @param power_ratios (可选) 功率分配比例，若为空则按各轮需求等比缩放
     */
    void solve(std::vector<MotorData>& motors,
               float referee_power_limit,
               float current_buffer_energy,
               const std::vector<float>* power_ratios = nullptr);

    // --- 遥测接口 (用于给 MATLAB 喂数据) ---
    /**
     * @brief 获取整车预测总功率 (可与裁判系统真实功率对比验证模型准确性)
     */
    float getTotalPredictedPower() const { return _last_total_predict; }

private:
    explicit PowerController(size_t motor_num);
    ~PowerController() = default;

    float updateDynamicLimit(float referee_limit, float buffer_energy);

    std::vector<FitParams> _params;
    BufferPidParams _pid_params;

    float _integral_err;
    float _last_err;
    float _last_total_predict;
};

} // namespace pyro

#endif