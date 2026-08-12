#include "pyro_17mm_booster.h"
#include "FreeRTOS.h"
#include "task.h"
// 6. 单发状态 (SHOOT_SINGLE_BULLET)
// 位置环推进一发, 堵转时进入校准

extern pyro::booster_cmd_t *booster_cmd_ptr;

namespace pyro
{

void shoot_17mm_control_t::state_single_bullet_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode         = data_ctx_t::trig_mode_e::POSITION; // 切位置模式
    ctx->_ctx.data.trig_pid_active   = true;
    // 目标角度: 在上一次目标基础上前进一发
    ctx->_ctx.data.target_trig_rad   = ctx->_ctx.data.target_trig_rad + PI / 4;
    ctx->_ctx.data.block_start_tick  = 0; // 清零堵转计时
}

void shoot_17mm_control_t::state_single_bullet_t::execute(owner *ctx)
{
    static uint16_t time = 0;
    time ++;
    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 计算当前角度误差 ---
    float err = ctx->_ctx.data.target_trig_rad - ctx->_ctx.data.current_trig_rad;

    // --- 堵转检测: 误差较大但速度极低 ---
    // 模板用转子角速度 < 10 rad/s，换算到拨弹盘为 10/36 ≈ 0.28 rad/s
    if (std::abs(err) > PI / 16.0f && std::abs(ctx->_ctx.data.current_trig_radps) < 0.3f)
    {
        if (ctx->_ctx.data.block_start_tick == 0)
        {
            ctx->_ctx.data.block_start_tick = xTaskGetTickCount();
        }
        else if (xTaskGetTickCount() - ctx->_ctx.data.block_start_tick >= pdMS_TO_TICKS(CALI_BLOCK_TIME_MS))
        {
            // 堵转超时 1000ms, 记录来源状态并进入校准
            ctx->_ctx.data.jam_source_state = data_ctx_t::state_e::SINGLE_BULLET;
            this->request_switch(&ctx->_state_cali_reverse);
            return;
        }
    }
    else
    {
        ctx->_ctx.data.block_start_tick = 0;
    }

    // --- 到达目标 → 完成 ---
    if (std::abs(err) < 0.003f || time > 500)
    {
        this->request_switch(&ctx->_state_done);
        time = 0;
    }
}

void shoot_17mm_control_t::state_single_bullet_t::exit(owner *ctx)
{
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();
    booster_cmd_ptr->single_shoot = false;
}

}
