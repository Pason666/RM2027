#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"

namespace pyro
{
void quad_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{
    owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
    owner->_ctx.data.target_trig_radps = 0;
    if (&owner->_state_active._homing_state == owner->_state_active._last_state)
    {
        owner->_ctx.data.target_trig_rad -= 0.67546f; // 待调整
    }
    else if (&owner->_state_active._stall_state ==
             owner->_state_active._last_state)
    {
        // 什么都不做
    }
    else
    {
        owner->_ctx.data.target_trig_rad += PI / 3; // 待调整
    }
}

void quad_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{

    // 回到合适角度后，切换回拨弹状态
    if (fabs(owner->_ctx.data.current_trig_rad -
             owner->_ctx.data.target_trig_rad) < 0.12f)
    {
        request_switch(&owner->_state_active._interim_state);
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}
} // namespace pyro