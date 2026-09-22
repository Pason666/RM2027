#include "pyro_referee.h"
#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"
#include "large_launcher_config.h"

namespace pyro
{
void tri_booster_t::fsm_active_t::state_ready_t::enter(owner *owner)
{
    owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
    owner->_ctx.data.fric_err = false;
    owner->_ctx.data.ready_state_flag = true;
    _fric_unready_start_time = 0.0f;
}

void tri_booster_t::fsm_active_t::state_ready_t::execute(owner *owner)
{
    constexpr float FRIC_SWITCH_BUFFER_MS = 50.0f;
    const float now_ms = dwt_drv_t::get_timeline_ms();

    if (owner->_ctx.cmd->fire_count != owner->_ctx.data.internal_fire_count)
    {
        owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
        owner->_ctx.data.signal_timer = dwt_drv_t::get_timeline_ms();

        // 从裁判系统获取热量数据
        auto *referee = referee_drv_t::get_instance();
        bool heat_ok = true;  // 默认允许发射

        if (referee->is_online())
        {
            const auto &referee_data = referee->get_data();
            uint16_t heat_limit = referee_data.robot_status.shooter_barrel_heat_limit;
            uint16_t heat = referee_data.power_heat.shooter_42mm_barrel_heat;

            if (0xFFFF == heat_limit || 0 == heat_limit)
            {
                heat_ok = true;  // 无限制或未初始化
            }
            else
            {
                if (heat_limit <= 110)
                {
                    heat_ok = (0 == heat);
                }
                else
                {
                    heat_ok = (heat + 110 < heat_limit);
                }
            }
        }

        if (heat_ok)
        {
            owner->_ctx.data.target_trig_rad =
                tri_booster_t::_get_next_trigger_preset(
                    owner->_ctx.data.current_trig_rad,
                    TRIGGER_PRESET_MIN_ADVANCE_RAD);

            request_switch(&owner->_state_active._busy_state);
        }
    }

    bool fric_unready = false;
    for (int i = 0; i < 3; i++)
    {
        if (abs(owner->_ctx.data.current_fric_mps[i] - owner->_ctx.data.target_fric_mps[i]) > 1.0f)
        {
            fric_unready = true;
            break;
        }
    }
    if (fric_unready)
    {
        if (_fric_unready_start_time == 0.0f)
        {
            _fric_unready_start_time = now_ms;
        }
        else if (now_ms - _fric_unready_start_time >= FRIC_SWITCH_BUFFER_MS)
        {
            request_switch(&owner->_state_active._interim_state);
        }
    }
    else
    {
        _fric_unready_start_time = 0.0f;
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void tri_booster_t::fsm_active_t::state_ready_t::exit(owner *owner)
{
    owner->_ctx.data.ready_state_flag = false;
}
} // namespace pyro
