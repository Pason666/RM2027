#include "pyro_target_robot.h"

namespace pyro
{

void target_robot_t::state_active_t::enter(owner *owner)
{
    // 使能所有电机
    owner->_ctx.motor.steering[0]->enable();
    owner->_ctx.motor.steering[1]->enable();
    owner->_ctx.motor.wheel[0]->enable();
    owner->_ctx.motor.wheel[1]->enable();
    owner->_ctx.motor.wheel[2]->enable();
    owner->_ctx.motor.wheel[3]->enable();
    owner->_ctx.motor.yaw->enable();
}

void target_robot_t::state_active_t::execute(owner *owner)
{
    // 1. 运动学解算 (含坐标系变换 + IMU锁头)
    owner->_kinematics_solve();

    // 2. PID 控制计算
    owner->_chassis_control();

    // 3. 发送电机扭矩命令
    owner->_send_motor_command();
}

void target_robot_t::state_active_t::exit(owner *owner)
{
    // 无需清理
}

} // namespace pyro
