#include "pyro_17mm_booster.h"
// 2. 摩擦轮启动状态 (SHOOT_READY_FRIC)
namespace pyro
{

void shoot_17mm_control_t::state_ready_fric_t::enter(owner *ctx)
{
    ctx->_ctx.data.target_fric_radps[0] = -lin_v_to_radps(TARGET_BULLET_SPEED);
    ctx->_ctx.data.target_fric_radps[1] = lin_v_to_radps(TARGET_BULLET_SPEED);
    ctx->_ctx.data.fric_pid_active      = true;
    ctx->_ctx.data.current_state        = data_ctx_t::state_e::READY_FRIC;
}
void shoot_17mm_control_t::state_ready_fric_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // 检查摩擦轮速度是否达标
    if (std::abs(ctx->_ctx.data.current_fric_radps[0] -
                 -lin_v_to_radps(TARGET_BULLET_SPEED)) < 150 &&
        std::abs(ctx->_ctx.data.current_fric_radps[1] -
                 lin_v_to_radps(TARGET_BULLET_SPEED)) < 150)
    {
        this->request_switch(&ctx->_state_ready_shoot);
    }
}
void shoot_17mm_control_t::state_ready_fric_t::exit(owner *ctx)
{
}

}