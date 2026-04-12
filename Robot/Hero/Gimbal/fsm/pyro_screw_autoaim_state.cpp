//
// Created by Clean on 2026/4/6.
//
#include "pyro_screw_gimbal.h"
#include "screw_config.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro
{
void screw_gimbal_t::fsm_active_t::autoaim_state_t::enter(owner *owner)
{
    // 切换到 Autoaim 模式时，对齐目标防止跳变
    owner->_ctx.data.target_yaw_rad   = owner->_ctx.data.yaw_imu_rad;
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_motor_rad;

    // 清除历史积分
    owner->_ctx.pid.pitch_pos->clear();
    owner->_ctx.pid.pitch_spd->clear();
    owner->_ctx.pid.yaw_pos->clear();
    owner->_ctx.pid.yaw_spd->clear();
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::execute(owner *owner)
{
    // ==========================================
    // 核心解算：世界坐标系 <-> 机械相对坐标系
    // ==========================================
    // 1. 直接读取由底盘四元数解算出的底盘真实 Pitch 倾角
    float chassis_pitch_world = owner->_ctx.data.chassis_pitch_rad;

    // 2. [解算 A] 把当前机械角度解到世界坐标系下的 pitch (满足你的理论映射)
    //    云台世界 Pitch = 云台机械角 + 底盘世界 Pitch
    owner->_ctx.data.current_pitch_motor_world = owner->_ctx.data.current_pitch_motor_rad + chassis_pitch_world;

    // 3. [解算 B] 把视觉下发的目标角度 (世界坐标系) 解到相对角 (机械角度)
    //    目标机械角 = 目标世界 Pitch - 底盘世界 Pitch
    float target_pitch_relative = owner->_ctx.cmd->target_pitch - chassis_pitch_world;


    // 自瞄模式下，直接读取外部传入的绝对目标角度并应用映射
    if (fabs(owner->_ctx.cmd->target_pitch) < PI &&
        fabs(owner->_ctx.cmd->target_yaw) < PI)
    {
        owner->_ctx.data.target_pitch_rad = target_pitch_relative;
        owner->_ctx.data.target_yaw_rad   = owner->_ctx.cmd->target_yaw;
    }
    else
    {
        // 视觉掉线或数据异常时的兜底步进
        owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
        owner->_ctx.data.target_yaw_rad   += owner->_ctx.cmd->yaw_delta_angle;
    }

    // ==========================================
    // 1. Pitch 相对角限幅 (绝对安全限制，防止丝杠撞到底)
    // ==========================================
    owner->_ctx.data.target_pitch_rad =
        std::clamp(owner->_ctx.data.target_pitch_rad, PITCH_MIN_RELATIVE_RAD,
                   PITCH_MAX_RELATIVE_RAD);

    // ==========================================
    // 2. Yaw 环化限幅 (-PI, PI)
    // ==========================================
    owner->_ctx.data.target_yaw_rad =
        pyro::loop_fp32_constrain(owner->_ctx.data.target_yaw_rad, -PI, PI);

    // ==========================================
    // 3. 执行常规的底层控制与发送指令
    // ==========================================
    owner->_gimbal_control();
    _send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::autoaim_state_t::exit(owner *owner)
{
}
} // namespace pyro