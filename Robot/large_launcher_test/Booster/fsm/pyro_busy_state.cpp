#include "pyro_large_launcher.h"

namespace pyro
{

void tri_booster_t::fsm_active_t::state_busy_t::enter(owner *owner)
{
}

void tri_booster_t::fsm_active_t::state_busy_t::execute(owner *owner)
{
    float error = owner->_ctx.data.target_trig_rad - owner->_ctx.data.current_trig_rad;
    error = tri_booster_t::_normalize_angle(error);

    if (abs(error) < 0.3f)
    {
        request_switch(&owner->_state_active._interim_state);
    }
    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void tri_booster_t::fsm_active_t::state_busy_t::exit(owner *owner)
{
}

}
