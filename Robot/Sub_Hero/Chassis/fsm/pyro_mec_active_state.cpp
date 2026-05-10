#include "pyro_mec_chassis.h"

namespace pyro
{
// --- Active State ---
void mec_chassis_t::state_active_t::enter(owner *owner)
{
    // 进入 Active 模式时，重置 PID
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.motor.wheels[i]->enable();
    }
    owner->_ctx.pid.follow_pid->clear();
    for(auto* pid : owner->_ctx.pid.wheel_pid) {
        pid->clear();
    }
}

void mec_chassis_t::state_active_t::execute(owner *owner)
{
    // 1. 运动学解算
    // (包含：矢量旋转变换 + 跟随 PID 计算 -> 输出 target_wheel_rpm)
    owner->_kinematics_solve();

    // 2. 闭环控制 (Input: target_rpm, current_rpm -> Output: torque)
    owner->_chassis_control(&owner->_ctx);

    // 3. 发送波形
    owner->_send_motor_command(&owner->_ctx);
}

void mec_chassis_t::state_active_t::exit(owner *owner)
{
    // 无特殊操作
}
} // namespace pyro