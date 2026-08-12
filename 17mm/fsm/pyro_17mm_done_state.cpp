#include "pyro_17mm_booster.h"
// 8. 射击完成状态 (SHOOT_DONE)

namespace pyro
{

void shoot_17mm_control_t::state_done_t::enter(owner *ctx)
{
    // --- 位置环锁位, 保持拨弹盘有力 ---
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.current_state      = data_ctx_t::state_e::DONE;
}
void shoot_17mm_control_t::state_done_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }
    this->request_switch(&ctx->_state_ready_shoot);
}
void shoot_17mm_control_t::state_done_t::exit(owner *ctx)
{
}

}