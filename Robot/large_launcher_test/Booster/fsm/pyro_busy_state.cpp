#include "pyro_large_launcher.h"
#include "pyro_dwt_drv.h"

namespace pyro
{

void tri_booster_t::fsm_active_t::state_busy_t::enter(owner *owner)
{
    _start_time_ms = dwt_drv_t::get_timeline_ms();
}

void tri_booster_t::fsm_active_t::state_busy_t::execute(owner *owner)
{
    constexpr float SINGLE_DONE_ANGLE_THRESHOLD = 0.05f;  // 约2.9度
    constexpr float SINGLE_DONE_TIMEOUT_MS = 300.0f;  // 降低超时到300ms
    constexpr float SINGLE_SHOT_ANGLE = 2.0f * 3.14159f / 3.0f;  // 三发一圈，约120度

    const float now_ms = dwt_drv_t::get_timeline_ms();

    float error = owner->_ctx.data.target_trig_rad - owner->_ctx.data.current_trig_rad;
    error = tri_booster_t::_normalize_angle(error);

    // 参考Launcher_test的到位判定
    bool angle_reached = abs(error) < SINGLE_DONE_ANGLE_THRESHOLD;
    bool timeout = (now_ms - _start_time_ms) > SINGLE_DONE_TIMEOUT_MS;
    bool stalled = (abs(error) < SINGLE_SHOT_ANGLE / 4.0f) &&  // 误差小于步长的1/4
                   (abs(owner->_ctx.data.current_trig_radps) < 0.5f) &&  // 速度很低
                   ((now_ms - _start_time_ms) > 150.0f);  // 已运动150ms

    if (angle_reached || timeout || stalled)
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
