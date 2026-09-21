#include "pyro_dm_motor_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"
#include "large_launcher_config.h"

namespace pyro
{

void tri_booster_t::fsm_active_t::on_enter(owner *owner)
{
    static_cast<dm_motor_drv_t *>(owner->_ctx.motor.trigger_wheel)
        ->clear_error();
    owner->_ctx.motor.trigger_wheel->enable();
    owner->_ctx.motor.fric_wheels[0]->enable();
    owner->_ctx.motor.fric_wheels[1]->enable();
    owner->_ctx.motor.fric_wheels[2]->enable();
    owner->_ctx.data.internal_reset_count = owner->_ctx.cmd->reset_count;
    owner->_ctx.data.internal_shoot_data_reset_count =
        owner->_ctx.cmd->shoot_data_reset_count;

    change_state(&_homing_state);
}

void tri_booster_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->reset_count != owner->_ctx.data.internal_reset_count)
    {
        owner->_ctx.data.internal_reset_count = owner->_ctx.cmd->reset_count;
        change_state(&_homing_state);
        return;
    }

    if (owner->_ctx.cmd->shoot_data_reset_count !=
        owner->_ctx.data.internal_shoot_data_reset_count)
    {
        owner->_ctx.data.internal_shoot_data_reset_count =
            owner->_ctx.cmd->shoot_data_reset_count;
        owner->_reset_active_shoot_data();
    }

    owner->_speed_control();

    if (owner->_ctx.cmd->fric_on)
    {
        auto &shoot_data = owner->_ctx.shoot_data;

        // 三个摩擦轮全部同向同速
        owner->_ctx.data.target_fric_mps[0] = shoot_data.fric_mps;
        owner->_ctx.data.target_fric_mps[1] = shoot_data.fric_mps;
        owner->_ctx.data.target_fric_mps[2] = shoot_data.fric_mps;

        owner->_fric_control();
    }
    else
    {
        owner->_ctx.data.target_fric_mps[0] = 0.0f;
        owner->_ctx.data.target_fric_mps[1] = 0.0f;
        owner->_ctx.data.target_fric_mps[2] = 0.0f;
        owner->_fric_control();
        for (int i = 0; i < 3; i++)
        {
            if (abs(owner->_ctx.data.current_fric_mps[i]) < 0.3f)
            {
                owner->_ctx.data.out_fric_torque[i] = 0.0f;
            }
        }
    }
    owner->_send_fric_command();

    owner->_launch_delay_calculate();

    // 堵转检测：通过拨弹盘电机力矩和速度判断
    constexpr float STALL_TIME_THRESHOLD   = 600.0f;
    constexpr float STALL_TORQUE_THRESHOLD = 5.0f;  // DM4310力矩阈值
    constexpr float STALL_SPEED_THRESHOLD  = 0.1f;

    static float stall_start_time       = 0.0f;
    static uint16_t clear_stall_counter = 0;

    if (abs(owner->_ctx.data.current_trig_radps) < STALL_SPEED_THRESHOLD &&
        abs(owner->_ctx.data.current_trig_torque) > STALL_TORQUE_THRESHOLD)
    {
        clear_stall_counter = 0;
        if (stall_start_time == 0.0f)
        {
            stall_start_time = dwt_drv_t::get_timeline_ms();
        }
        else
        {
            const float elapsed_time =
                dwt_drv_t::get_timeline_ms() - stall_start_time;
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
        if (stall_start_time != 0.0f)
        {
            clear_stall_counter++;
            if (clear_stall_counter >= 8)
            {
                stall_start_time    = 0.0f;
                clear_stall_counter = 0;
            }
        }
        else
        {
            clear_stall_counter = 0;
        }
    }
}

void tri_booster_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro
