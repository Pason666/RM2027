#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"


namespace pyro
{
void tri_booster_t::fsm_active_t::state_interim_t::enter(owner *owner)
{
    _ready_wait_start_time  = dwt_drv_t::get_timeline_ms();
    _fric_ready_start_time  = 0.0f;
    owner->_ctx.data.fric_err = false;

    // 在进入interim时同步fire_count，避免在ready时丢失
    owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
}

void tri_booster_t::fsm_active_t::state_interim_t::execute(owner *owner)
{
    constexpr float FRIC_READY_TIMEOUT_MS = 1000.0f;
    constexpr float FRIC_SWITCH_BUFFER_MS = 50.0f;
    const float now_ms = dwt_drv_t::get_timeline_ms();

    // 三个摩擦轮转速判断 - 已关闭
    const bool fric_ready = true;

    if (fric_ready)
    {
        if (_fric_ready_start_time == 0.0f)
        {
            _fric_ready_start_time = now_ms;
        }
        else if (now_ms - _fric_ready_start_time >= FRIC_SWITCH_BUFFER_MS)
        {
            request_switch(&owner->_state_active._ready_state);
        }
    }
    else
    {
        _fric_ready_start_time = 0.0f;
    }

    if (owner->_ctx.cmd->fric_on)
    {
        const float elapsed_time = now_ms - _ready_wait_start_time;
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

    // 拨弹盘保持目标位置（不要改变target，让它保持为计算出的位置）
    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void tri_booster_t::fsm_active_t::state_interim_t::exit(owner *owner)
{
}
} // namespace pyro
