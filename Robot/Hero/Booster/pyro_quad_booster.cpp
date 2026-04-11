#include "pyro_quad_booster.h"
#include "pyro_algo_common.h"
#include "pyro_com_canrx.h"
#include "pyro_dwt_drv.h"
#include <cmath>
#include "quad_config.h"

#include <algorithm>

namespace pyro
{

quad_booster_t::quad_booster_t() : module_base_t("quad_booster")
{
    _ctx = {};
}

status_t quad_booster_t::_init()
{

    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid = _module_deps.pid_deps;
    // 3. 弹速控制初始化
    can_rx_drv_t::subscribe(can_hub_t::can1, 0x135);
    _ctx.pid.ball_speed_pid = new pid_t(0.32f, 0.0f, 0.005f, 0.0f, 2.0f);

    return PYRO_OK;
}

float quad_booster_t::_normalize_angle(float angle)
{
    // 归一化到 [-PI, PI]
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

void quad_booster_t::_update_feedback()
{
    // 1. 摩擦轮反馈
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_torque[i] = _ctx.motor.fric_wheels[i]->get_current_torque();
    }
    _ctx.data.current_fric_mps[0] = _ctx.motor.fric_wheels[0]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[1] = _ctx.motor.fric_wheels[1]->get_current_rotate() * FRIC1_RADIUS;
    _ctx.data.current_fric_mps[2] = _ctx.motor.fric_wheels[2]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[3] = _ctx.motor.fric_wheels[3]->get_current_rotate() * FRIC1_RADIUS;

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.abs_current_fric_mps[i] = abs(_ctx.data.current_fric_mps[i]);
    }


    // 2. 拨弹反馈
    _ctx.motor.trigger_wheel->update_feedback();

    // --- A. 速度反馈 ---
    _ctx.data.current_trig_radps =
        _ctx.motor.trigger_wheel->get_current_rotate();

    // --- B. 扭矩反馈 ---
    _ctx.data.current_trig_torque =
        _ctx.motor.trigger_wheel->get_current_torque();

    // --- C. 角度反馈 (-PI ~ PI) ---
    _ctx.data.current_trig_rad = _ctx.motor.trigger_wheel->get_current_position();
}

void quad_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void quad_booster_t::_speed_control()
{
    std::array<uint8_t, 8> raw_data{};

    // 仅在成功接收到新弹速的这一帧，才进行闭环计算
    if (can_rx_drv_t::get_data(pyro::can_hub_t::can1, 0x135, raw_data))
    {
        // 1. 更新弹速历史数据
        _ctx.shoot_data.ball_speed[2] = _ctx.shoot_data.ball_speed[1];
        _ctx.shoot_data.ball_speed[1] = _ctx.shoot_data.ball_speed[0];
        _ctx.shoot_data.ball_speed[0] =
            *reinterpret_cast<float *>(raw_data.data());

        for (int i = 0; i < 3; i++)
        {
            if (_ctx.shoot_data.ball_speed[i] == 0.0f)
            {
                _ctx.shoot_data.ball_speed[i] = _ctx.cmd->target_speed;
            }
        }

        // 2. 确保目标弹速有效，避免启动时出现误动作
        if (_ctx.cmd->target_speed > 7.5f)
        {
            // --- A. 定义近期弹速的权重 ---
            // 越新的弹速参考价值越大
            constexpr float w0 = 0.72f; // 最新一发
            constexpr float w1 = 0.21f; // 上一发
            constexpr float w2 = 0.07f; // 上上发

            // --- B. 计算带符号的均方误差 ---
            float e0 = _ctx.shoot_data.ball_speed[0] - _ctx.cmd->target_speed;
            float e1 = _ctx.shoot_data.ball_speed[1] - _ctx.cmd->target_speed;
            float e2 = _ctx.shoot_data.ball_speed[2] - _ctx.cmd->target_speed;

            // 采用 e * |e| 保留误差方向 (加速或减速)
            float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                        (w1 * e1 * std::abs(e1)) +
                                        (w2 * e2 * std::abs(e2));

            // --- C. PID 计算速度增量 ---
            // 由于 signed_weighted_mse 本身已经是误差值，直接将其作为
            // target，current 设为 0
            float speed_increment =
                _ctx.pid.ball_speed_pid->calculate(0.0f, signed_weighted_mse);

            // --- D. 累加到 fric1 的基础转速上 ---
            _ctx.shoot_data.fric1_mps += speed_increment;

            // --- E. 安全限幅 (非常重要) ---
            // 避免闭环异常导致单侧摩擦轮转速过高或过低，导致卡弹或弹道严重偏斜
            // 这里的限幅值请根据你实际的摩擦轮物理极限进行调整
            constexpr float MAX_FRIC1_MPS = 16.0f;
            constexpr float MIN_FRIC1_MPS = 9.0f;

            if (_ctx.shoot_data.fric1_mps > MAX_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MAX_FRIC1_MPS;
            }
            else if (_ctx.shoot_data.fric1_mps < MIN_FRIC1_MPS)
            {
                _ctx.shoot_data.fric1_mps = MIN_FRIC1_MPS;
            }
        }
    }
}

void quad_booster_t::_launch_delay_calculate()
{
    // 2. 发弹延迟计算
    // 通过外级摩擦轮转速和扭矩判断是否发弹
    // 计算信号发生时间（在ready状态中获取）到当前时间的差值

    _ctx.data.fresh_timer++;

    if (_ctx.shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[1]) > 0.8f &&
        _ctx.shoot_data.fric1_mps - std::abs(_ctx.data.current_fric_mps[3]) > 0.8f &&
        std::abs(_ctx.data.current_fric_torque[1]) > 3.0f &&
        std::abs(_ctx.data.current_fric_torque[2]) > 3.0f &&
        _ctx.data.fresh_timer > 220)
    {
        _ctx.data.launch_delay_timer[2] = _ctx.data.launch_delay_timer[1];
        _ctx.data.launch_delay_timer[1] = _ctx.data.launch_delay_timer[0];
        _ctx.data.launch_delay_timer[0] = (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer > 200.0f) ? _ctx.data.avg_launch_delay : (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer + 20.0f);

        _ctx.data.avg_launch_delay = 0.7f * _ctx.data.launch_delay_timer[0] + 0.2f * _ctx.data.launch_delay_timer[1] + 0.1f * _ctx.data.launch_delay_timer[2];
        _ctx.data.fresh_timer = 0;
    }
}

void quad_booster_t::_fric_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void quad_booster_t::_trigger_position_control()
{
    // 1. 计算原始误差
    float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;

    // 2. 处理过零点问题，将误差归一化到 [-PI, PI]，选择最短路径
    error = _normalize_angle(error);

    // 3. 拨弹 PID 计算：以误差作为目标值，当前值设为 0
    _ctx.data.target_trig_radps = _ctx.pid.trigger_pos_pid->calculate(error, 0.0f);

    // --- 引入拨弹前馈补偿 ---
    static float ff_torque = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 1.0f; // 速度死区 (rad/s)，防止在目标位置附近静止时产生力矩抖动
    constexpr float TRIG_FF_TORQUE = 0.505f;       // 前馈力矩大小

    // 判断逻辑：目标速度为负（代表正往出拨弹），且超过速度死区
    // 因为往出拨弹是负方向，所以要施加同方向的负向力矩 (-0.505)
    if (_ctx.data.target_trig_radps < -TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = -TRIG_FF_TORQUE;
    }
    else if (_ctx.data.target_trig_radps > 0.0f)
    {
        ff_torque = 0.0f;
    }

    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps) + ff_torque;

    _ctx.data.out_trig_torque = std::clamp(_ctx.data.out_trig_torque , -7.0f, 7.0f);
}

void quad_booster_t::_trigger_speed_control()
{
    // --- 引入拨弹前馈补偿 ---
    float ff_torque = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 0.5f; // 速度死区 (rad/s)
    constexpr float TRIG_FF_TORQUE = 0.505f;       // 前馈力矩大小

    // 判断逻辑：目标速度为负（代表正往出拨弹），且超过速度死区
    if (_ctx.data.target_trig_radps < -TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = -TRIG_FF_TORQUE;
    }

    _ctx.data.out_trig_torque = _ctx.pid.trigger_spd_pid->calculate(
        _ctx.data.target_trig_radps, _ctx.data.current_trig_radps) + ff_torque;
}

void quad_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i] + 0.1f * _ctx.data.current_fric_torque[i]);
    }
}

void quad_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

quad_booster_t::booster_ctx_t quad_booster_t::get_ctx() const
{
    return _ctx;
}

} // namespace pyro