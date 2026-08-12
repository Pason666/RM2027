#include "pyro_17mm_booster.h"
#include "FreeRTOS.h"
#include "task.h"
// 校准反转状态 (CALI_REVERSE)
// 拨弹盘高速反转, 直到碰到机械死区 (堵转) 后切换到 CaliForward

namespace pyro
{

void shoot_17mm_control_t::state_cali_reverse_t::enter(owner *ctx)
{
    // --- 初始化 ---
    ctx->_ctx.data.block_start_tick    = 0;
    ctx->_ctx.data.trig_mode           = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_radps   = CALI_REVERSE_RADPS;
    ctx->_ctx.data.trig_pid_active     = true;
    ctx->_ctx.data.current_state       = data_ctx_t::state_e::CALI_REVERSE;
}

void shoot_17mm_control_t::state_cali_reverse_t::execute(owner *ctx)
{
    // --- 高速反转寻找机械死区 ---
    ctx->_ctx.data.target_trig_radps = CALI_REVERSE_RADPS;

    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 堵转检测: 实际速度远小于目标 → 碰到机械限位 ---
    float speed_error = std::abs(ctx->_ctx.data.current_trig_radps - ctx->_ctx.data.target_trig_radps);
    if (speed_error > std::abs(ctx->_ctx.data.target_trig_radps) * CALI_BLOCK_THRESHOLD)
    {
        if (ctx->_ctx.data.block_start_tick == 0)
        {
            ctx->_ctx.data.block_start_tick = xTaskGetTickCount();
        }
        else if (xTaskGetTickCount() - ctx->_ctx.data.block_start_tick >= pdMS_TO_TICKS(CALI_BLOCK_TIME_MS))
        {
            // 堵转超时, 根据热量状态决定目标并切换到正转校准
            switch (ctx->_ctx.data.jam_source_state)
            {
                case data_ctx_t::state_e::SINGLE_BULLET:
                    ctx->_ctx.data.target_state_after_cali = data_ctx_t::state_e::SINGLE_BULLET;
                    break;
                case data_ctx_t::state_e::CONTINUE_BULLET:
                    if (ctx->_ctx.data.heatController.isApproachingHeatLimit())
                    {
                        // 热量紧张 → 回到就绪, 不再激进连发
                        ctx->_ctx.data.target_state_after_cali = data_ctx_t::state_e::READY_SHOOT;
                    }
                    else
                    {
                        ctx->_ctx.data.target_state_after_cali = data_ctx_t::state_e::CONTINUE_BULLET;
                    }
                    break;
                case data_ctx_t::state_e::READY_SHOOT:
                    ctx->_ctx.data.target_state_after_cali = data_ctx_t::state_e::SINGLE_BULLET;
                    break;
                default:
                    ctx->_ctx.data.target_state_after_cali = data_ctx_t::state_e::READY_SHOOT;
                    break;
            }
            this->request_switch(&ctx->_state_cali_forward);
        }
    }
    else
    {
        ctx->_ctx.data.block_start_tick = 0;
    }
}

void shoot_17mm_control_t::state_cali_reverse_t::exit(owner *ctx)
{
    // --- 清空速度环积分, 防止残留积分继续驱动电机 ---
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();

    // --- 以当前编码器位置作为新的零点偏移 ---
    ctx->_ctx.data.trigger_offset = ctx->_ctx.data.current_trig_rad;
}

} // namespace pyro