#include "pyro_mec_chassis.h"

namespace pyro
{
void mec_chassis_t::state_passive_t::enter(owner *owner)
{
    // 清零目标
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.motor.wheels[i]->disable();
    }
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0;
        owner->_ctx.data.out_wheel_torque[i] = 0;
    }
    // 清除跟随 PID 状态
    owner->_ctx.pid.follow_pid->clear();
    for (auto *pid : owner->_ctx.pid.wheel_pid)
    {
        pid->clear();
    }
}

void mec_chassis_t::state_passive_t::execute(owner *owner)
{
    // 持续输出 0 力矩
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0;
    }
    _send_motor_command(&owner->_ctx);
}

void mec_chassis_t::state_passive_t::exit(owner *owner)
{

}
} // namespace pyro