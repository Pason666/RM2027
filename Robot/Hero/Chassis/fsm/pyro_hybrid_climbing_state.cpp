#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_enter(owner *owner)
{
    // 进入爬坡模式，重新初始化所有驱动机构的 PID
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }

    // --- 新增：初始化测距滤波与触发器状态 ---
    _filtered_distance = owner->_ctx.data.distance_mm;
    _is_guide_wheel_suspended = false;
    _auto_retract_flag = false;
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_execute(owner *owner)
{
    // 1. 麦轮速度环控制 (提供前轮牵引力)
    owner->_mecanum_control();

    // 2. 履带速度环控制 (履带正式介入，提供主要越障/爬坡推进力)
    owner->_track_control();


    // ================= 自动收腿：施密特触发器逻辑 =================

    // 1. 一阶低通滤波 (LPF) 剔除毛刺，滤波参数见 config.h
    _filtered_distance = CLIMB_DIST_LPF_ALPHA * owner->_ctx.data.distance_mm +
                         (1.0f - CLIMB_DIST_LPF_ALPHA) * _filtered_distance;

    // 2. 状态转移逻辑
    if (_filtered_distance > CLIMB_DIST_THRES_HIGH)
    {
        // 冲破上限，记录导轮悬空状态 (车头正在翘起或跨越中)
        _is_guide_wheel_suspended = true;
    }
    else if (_filtered_distance < CLIMB_DIST_THRES_LOW && _is_guide_wheel_suspended)
    {
        // 跌破下限，且之前确实处于悬空状态 -> 证明导轮已经稳稳跨上台阶！
        // 结合运动学正解算的实际底盘前进速度，过滤掉可能原地的误判
        if (owner->_ctx.data.real_vx > CLIMB_VEL_X_THRES)
        {
            _auto_retract_flag = true; // 锁存自动收腿信号
            _is_guide_wheel_suspended = false; // 触发后复位悬空状态
        }
    }
    // =============================================================

    // 综合原有的手动控制与自动判定
    if (owner->_ctx.cmd->leg_retract || _auto_retract_flag)
    {
        change_state(&leg_retraction_state);
    }
    else
    {
        change_state(&track_climbing_state);
    }
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::on_exit(owner *owner)
{
    // 退出爬坡模式时的清理工作（当前可留空）
}

} // namespace pyro
