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

    owner->_ctx.motor.track[0]  ->enable();
    owner->_ctx.motor.track[1]  ->enable();
    owner->_ctx.motor.leg[0]    ->enable();
    owner->_ctx.motor.leg[1]    ->enable();

    // --- 初始化测距滤波与触发器状态 ---
    _filtered_distance = owner->_ctx.data.distance_mm;
    _is_guide_wheel_suspended = false;
    _auto_retract_flag = false;
    _retract_hold_tick = 0; // 初始化计时器
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
        _retract_hold_tick = 0; // 悬空时清空退出计时
    }
    else if (_filtered_distance < CLIMB_DIST_THRES_LOW)
    {
        if (_is_guide_wheel_suspended && owner->_ctx.data.real_vx < -CLIMB_VEL_X_BACK_THRES)
        {
            _is_guide_wheel_suspended = false;
        }
        // a. 进入自动收腿的判定
        else if (_is_guide_wheel_suspended && owner->_ctx.data.real_vx > CLIMB_VEL_X_FRONT_THRES)
        {
            _auto_retract_flag = true; // 锁存自动收腿信号
            _is_guide_wheel_suspended = false; // 触发后复位悬空状态
            _retract_hold_tick = 0; // 刚触发时清零计时器
        }

        // b. 退出自动收腿的判定
        if (_auto_retract_flag)
        {
            _retract_hold_tick++; // 只要低于下限且处于收腿锁存期，就持续累加

            if (_retract_hold_tick >= CLIMB_RETRACT_HOLD_TICKS)
            {
                // 持续时间达标，证明已经稳稳在台阶面上开了一段距离了，退出收腿
                _auto_retract_flag = false;
                _retract_hold_tick = 0;
            }
        }
    }
    else // 距离位于上下限之间，或者异常突变高于下限
    {
        // 如果在自动收腿过程中距离突然变大，说明可能遇到坑洼或者并没有完全跨过去
        // 此时打断退出计时，要求必须连续贴地才允许退出
        if (_auto_retract_flag)
        {
            _retract_hold_tick = 0;
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
    // 退出爬坡模式时的清理工作
    owner->_ctx.data.out_leg_torque[0] = 0;
    owner->_ctx.data.out_leg_torque[1] = 0;
    owner->_ctx.data.out_track_torque[0] = 0;
    owner->_ctx.data.out_track_torque[1] = 0;

    owner->_ctx.motor.track[0]  ->disable();
    owner->_ctx.motor.track[1]  ->disable();
    owner->_ctx.motor.leg[0]    ->disable();
    owner->_ctx.motor.leg[1]    ->disable();
}

} // namespace pyro