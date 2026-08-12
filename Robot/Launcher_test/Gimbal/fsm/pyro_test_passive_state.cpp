#include "pyro_test_robot.h"

namespace pyro
{

void test_robot_t::state_passive_t::enter(owner *o)
{
    o->_ctx.motor.yaw->disable();
    o->_ctx.motor.pitch->disable();
    o->_ctx.motor.friction[0]->disable();
    o->_ctx.motor.friction[1]->disable();
    o->_ctx.motor.feeder->disable();
}

void test_robot_t::state_passive_t::execute(owner *o)
{
    o->_ctx.motor.yaw->send_torque(0);
    o->_ctx.motor.pitch->send_torque(0);
    o->_ctx.motor.friction[0]->send_torque(0);
    o->_ctx.motor.friction[1]->send_torque(0);
    o->_ctx.motor.feeder->send_torque(0);
}

void test_robot_t::state_passive_t::exit(owner * /*o*/)
{
}

} // namespace pyro
