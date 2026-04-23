#include "pyro_quad_booster.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void quad_booster_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.trigger_wheel->enable();
    owner->_ctx.motor.fric_wheels[0]->enable();
    owner->_ctx.motor.fric_wheels[1]->enable();
    owner->_ctx.motor.fric_wheels[2]->enable();
    owner->_ctx.motor.fric_wheels[3]->enable();

    change_state(&_homing_state);
}

void quad_booster_t::fsm_active_t::on_execute(owner *owner)
{
    // 1. 弹速闭环更新
    owner->_speed_control();

    if (owner->_ctx.cmd->fric_on)
    {
        // 根据吊射模式切换数据引用
        auto &shoot_data = owner->_ctx.cmd->sling_mode ? owner->_ctx.shoot_sling_data : owner->_ctx.shoot_normal_data;

        owner->_ctx.data.target_fric_mps[0] = shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[2] = -shoot_data.fric2_mps;
        owner->_ctx.data.target_fric_mps[1] = shoot_data.fric1_mps;
        owner->_ctx.data.target_fric_mps[3] = -shoot_data.fric1_mps;

        owner->_fric_control();
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[2] = 0.0f;
        owner->_ctx.data.target_fric_mps[3] = 0.0f;
        owner->_fric_control();
        for (int i = 0; i < 4; i++)
        {
            if (abs(owner->_ctx.data.current_fric_mps[i]) < 0.3f)
                owner->_ctx.data.out_fric_torque[i] = 0.0f;
        }
    }
    owner->_send_fric_command();

    // 2. 发弹延迟计算
    owner->_launch_delay_calculate();

    // 3. 拨弹盘堵转判断
    constexpr float STALL_TIME_THRESHOLD   = 400.0f;
    constexpr float STALL_TORQUE_THRESHOLD = 3.0f;
    constexpr float STALL_SPEED_THRESHOLD  = 0.4f;

    static float stall_start_time          = 0.0f;
    if (abs(owner->_ctx.data.current_trig_radps) < STALL_SPEED_THRESHOLD &&
        abs(owner->_ctx.data.current_trig_torque) > STALL_TORQUE_THRESHOLD)
    {
        if (stall_start_time == 0.0f)
        {
            stall_start_time = dwt_drv_t::get_timeline_ms();
        }
        else
        {
            const float elapsed_time = dwt_drv_t::get_timeline_ms() - stall_start_time;
            if (elapsed_time >= STALL_TIME_THRESHOLD)
            {
                change_state(&_stall_state);
                if (_active_state == &_stall_state)
                {
                    reset();
                }
                stall_start_time = 0.0f;
            }
        }
    }
    else
    {
        stall_start_time = 0.0f;
    }
}

void quad_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro