#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"

namespace pyro
{
void quad_booster_t::fsm_active_t::state_ready_t::enter(owner *owner)
{
    // 进入 ready 状态时，强制同步内部计数器与外部命令，
    // 清除在非 ready 状态期间（如 interim, busy, stall）累积的所有误触发开火指令。
    owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;
}

void quad_booster_t::fsm_active_t::state_ready_t::execute(owner *owner)
{
    // 检查命令计数器是否与内部追踪计数器不一致，不一致说明有新的开火请求
    if (owner->_ctx.cmd->fire_count != owner->_ctx.data.internal_fire_count)
    {
        // 立即同步计数器，防止重复发弹或连发
        owner->_ctx.data.internal_fire_count = owner->_ctx.cmd->fire_count;

        owner->_ctx.data.signal_timer = dwt_drv_t::get_timeline_ms();
        owner->_ctx.data.target_trig_rad -= PI / 3.0f; // 每次拨弹60度

        // 新增：每次改变目标值后立即归一化
        owner->_ctx.data.target_trig_rad = quad_booster_t::_normalize_angle(owner->_ctx.data.target_trig_rad);

        request_switch(&owner->_state_active._busy_state);
    }

    // @TODO: 循环判断摩擦轮转速，不符合要求则退回interim状态
    for (int i = 0; i < 4; i++)
    {
        if (abs(owner->_ctx.data.current_fric_mps[i] - owner->_ctx.data.target_fric_mps[i]) > 0.5f)
        {
            request_switch(&owner->_state_active._interim_state);
            break;
        }
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_ready_t::exit(owner *owner)
{

}
} // namespace pyro