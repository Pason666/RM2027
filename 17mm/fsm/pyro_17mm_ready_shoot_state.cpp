#include "pyro_17mm_booster.h"
// 3. 发射就绪状态 (SHOOT_READY_SHOOT)
namespace pyro
{

void shoot_17mm_control_t::state_ready_shoot_t::enter(owner *ctx)
{
    ctx->_ctx.data.fric_pid_active    = true;
    // --- 位置环锁位, 保持拨弹盘有力 ---
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.trig_pid_active    = true;
    ctx->_ctx.data.current_state      = data_ctx_t::state_e::READY_SHOOT;
}

void shoot_17mm_control_t::state_ready_shoot_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 单发触发: 未校准先进入校准, 热量不足则忽略 ---
    if (ctx->_ctx.cmd->single_shoot)
    {
        if (!ctx->_ctx.data.is_calibrated)
        {
            ctx->_ctx.data.jam_source_state = data_ctx_t::state_e::READY_SHOOT;
            this->request_switch(&ctx->_state_cali_reverse);
        }
        else if (!ctx->_ctx.cmd->heat_control_on || ctx->_ctx.data.heatController.canShootSingle())
        {
            this->request_switch(&ctx->_state_single_bullet);
        }
        return;
    }

    // --- 连发触发: 热量不足则忽略 ---
    if (ctx->_ctx.cmd->continue_shoot)
    {
        const bool can_continue =
            !ctx->_ctx.cmd->heat_control_on ||
            ctx->_ctx.data.heatController.canShootSingle();

        if (can_continue)
        {
#if !TRIGGER_CONTINUE_HEAT_RECOVERY_CALI_EN
            if (ctx->_ctx.cmd->heat_control_on &&
                ctx->_ctx.data.continue_heat_limited)
            {
                ctx->_ctx.data.suppress_continue_recovery_cali = true;
                ctx->_ctx.data.block_start_tick = 0;
            }
#endif
            ctx->_ctx.data.continue_heat_limited = false;
            this->request_switch(&ctx->_state_continue_bullet);
        }
        else
        {
            ctx->_ctx.data.continue_heat_limited = true;
            ctx->_ctx.data.suppress_continue_recovery_cali = false;
            ctx->_ctx.data.block_start_tick = 0;
        }
    }
    else
    {
        ctx->_ctx.data.continue_heat_limited = false;
        ctx->_ctx.data.suppress_continue_recovery_cali = false;
    }
}

void shoot_17mm_control_t::state_ready_shoot_t::exit(owner *ctx)
{
}

} // namespace pyro
