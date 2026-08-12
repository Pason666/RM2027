#include "pyro_17mm_booster.h"
// 1. 停止状态 (SHOOT_STOP)
namespace pyro
{

void shoot_17mm_control_t::state_stop_t::enter(owner *ctx)
{
    ctx->_ctx.data.target_fric_radps[0] = 0;
    ctx->_ctx.data.target_fric_radps[1] = 0;
    ctx->_ctx.data.fric_pid_active      = true;

    ctx->_ctx.data.trig_mode            = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps    = 0;
    ctx->_ctx.data.trig_pid_active      = true;
    ctx->_ctx.data.current_state        = data_ctx_t::state_e::STOP;
}

void shoot_17mm_control_t::state_stop_t::execute(owner *ctx)
{
    if (ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_ready_fric);
    }

    if (std::abs(ctx->_ctx.data.current_fric_radps[0]) < 10 &&
        std::abs(ctx->_ctx.data.current_fric_radps[1]) < 10)
        ctx->_ctx.data.fric_pid_active = false;

    if (std::abs(ctx->_ctx.data.current_trig_radps) < 0.01f)
        ctx->_ctx.data.trig_pid_active = false;
}

void shoot_17mm_control_t::state_stop_t::exit(owner *ctx)
{
    // 强制重置模式和目标，防止旧数据残留
    ctx->_ctx.data.trig_mode         = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps = 0;

    // --- 清除校准标志，每次从 stop 进入都需要重新校准 ---
    ctx->_ctx.data.is_calibrated     = false;
}

}
