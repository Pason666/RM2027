#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"


namespace pyro
{
void quad_booster_t::fsm_active_t::state_interim_t::enter(owner *owner)
{
    _ready_wait_start_time  = dwt_drv_t::get_timeline_ms();
    owner->_ctx.data.fric_err = false;
}

void quad_booster_t::fsm_active_t::state_interim_t::execute(owner *owner)
{
    constexpr float FRIC_READY_TIMEOUT_MS = 3000.0f;

    // 循环判断摩擦轮转速，符合要求则进入ready状态
    if (abs(owner->_ctx.data.current_fric_mps[0] -
            owner->_ctx.data.target_fric_mps[0]) < 0.5f &&
        abs(owner->_ctx.data.current_fric_mps[1] -
            owner->_ctx.data.target_fric_mps[1]) < 0.5f &&
        abs(owner->_ctx.data.current_fric_mps[2] -
            owner->_ctx.data.target_fric_mps[2]) < 0.5f &&
        abs(owner->_ctx.data.current_fric_mps[3] -
            owner->_ctx.data.target_fric_mps[3]) < 0.5f &&
        owner->_ctx.cmd->fric_on)
    {
        request_switch(&owner->_state_active._ready_state);
    }

    if (owner->_ctx.cmd->fric_on)
    {
        const float elapsed_time =
            dwt_drv_t::get_timeline_ms() - _ready_wait_start_time;
        if (elapsed_time >= FRIC_READY_TIMEOUT_MS)
        {
            owner->_ctx.data.fric_err = true;
        }
    }
    else
    {
        _ready_wait_start_time  = dwt_drv_t::get_timeline_ms();
        owner->_ctx.data.fric_err = false;
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_interim_t::exit(owner *owner)
{
}
} // namespace pyro