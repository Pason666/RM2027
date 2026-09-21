#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"

namespace pyro
{

void tri_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{
}

void tri_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{
    owner->_ctx.data.target_trig_radps = 0.0f;
    owner->_ctx.data.out_trig_torque = 0.0f;
    owner->_send_trigger_command();
}

void tri_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}

}
