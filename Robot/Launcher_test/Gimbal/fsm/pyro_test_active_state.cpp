#include "pyro_test_robot.h"

namespace pyro
{

void test_robot_t::state_active_t::enter(owner *o)
{
    o->_ctx.motor.pitch->enable();
    o->_ctx.motor.friction[0]->enable();
    o->_ctx.motor.friction[1]->enable();
    o->_ctx.motor.feeder->enable();

    // 上电时: 将目标位置初始化为passive状态记录的位置
    o->_ctx.data.target_pitch_rad = o->_ctx.data.pitch_init_position;
    o->_ctx.pid.pitch_pos_pid->clear();
    o->_ctx.pid.pitch_spd_pid->clear();

    // 重新激活摩擦轮 PID (若之前被两阶段停机切为零力矩)
    o->_ctx.data.fric_pid_active = true;

    // 从 passive 进入 active 时: 重置拨弹盘子状态机, 清除校准
    o->_ctx.data.is_calibrated       = false;
    o->_ctx.data.block_start_tick    = 0;
    o->_ctx.data.cmd_feeder_trigger  = false;
    o->_ctx.data.cmd_fire_enable     = false;
    o->_ctx.data.trig_state = test_robot_data_ctx_t::trig_state_e::READY;
    o->_trig_ready_enter();
}

void test_robot_t::state_active_t::execute(owner *o)
{
    o->_gimbal_control();
    o->_friction_control();
    o->_feeder_control();
    o->_send_motor_command();
}

void test_robot_t::state_active_t::exit(owner * /*o*/)
{
}

} // namespace pyro
