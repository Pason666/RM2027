#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"

// 调试变量
extern "C" {
    float g_debug_interim_target = 0.0f;
    float g_debug_interim_current = 0.0f;
    float g_debug_interim_error = 0.0f;
    float g_debug_interim_torque = 0.0f;
    uint8_t g_debug_in_interim = 0;
}

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
    g_debug_in_interim = 1;

    constexpr float FRIC_READY_TIMEOUT_MS = 1000.0f;
    constexpr float FRIC_SWITCH_BUFFER_MS = 50.0f;
    const float now_ms = dwt_drv_t::get_timeline_ms();

    // 调试：记录目标、当前位置
    g_debug_interim_target = owner->_ctx.data.target_trig_rad;
    g_debug_interim_current = owner->_ctx.data.current_trig_rad;
    float error = owner->_ctx.data.target_trig_rad - owner->_ctx.data.current_trig_rad;
    g_debug_interim_error = tri_booster_t::_normalize_angle(error);
    g_debug_interim_torque = owner->_ctx.data.out_trig_torque;

    // 三个摩擦轮转速判断 - 已关闭
    const bool fric_ready = true;
    // const bool fric_ready =
    //     abs(owner->_ctx.data.current_fric_mps[0] -
    //         owner->_ctx.data.target_fric_mps[0]) < 1.0f &&
    //     abs(owner->_ctx.data.current_fric_mps[1] -
    //         owner->_ctx.data.target_fric_mps[1]) < 1.0f &&
    //     abs(owner->_ctx.data.current_fric_mps[2] -
    //         owner->_ctx.data.target_fric_mps[2]) < 1.0f &&
    //     owner->_ctx.cmd->fric_on;

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
