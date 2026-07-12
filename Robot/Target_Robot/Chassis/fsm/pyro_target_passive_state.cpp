#include "pyro_target_robot.h"

namespace pyro
{

void target_robot_t::state_passive_t::enter(owner *owner)
{
    // 失能所有电机
    owner->_ctx.motor.steering[0]->disable();
    owner->_ctx.motor.steering[1]->disable();
    owner->_ctx.motor.wheel[0]->disable();
    owner->_ctx.motor.wheel[1]->disable();
    owner->_ctx.motor.wheel[2]->disable();
    owner->_ctx.motor.wheel[3]->disable();
    owner->_ctx.motor.yaw->disable();
}

void target_robot_t::state_passive_t::execute(owner *owner)
{
    // 发送零扭矩，确保电机不输出力
    owner->_ctx.motor.steering[0]->send_torque(0);
    owner->_ctx.motor.steering[1]->send_torque(0);
    owner->_ctx.motor.wheel[0]->send_torque(0);
    owner->_ctx.motor.wheel[1]->send_torque(0);
    owner->_ctx.motor.wheel[2]->send_torque(0);
    owner->_ctx.motor.wheel[3]->send_torque(0);
    owner->_ctx.motor.yaw->send_torque(0);
}

void target_robot_t::state_passive_t::exit(owner *owner)
{
    // 无需清理
}

} // namespace pyro
