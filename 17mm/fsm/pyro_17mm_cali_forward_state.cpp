#include "pyro_17mm_booster.h"
// 校准正转状态 (CALI_FORWARD)
// 从机械死区正转回到零点, 完成后根据目标状态恢复

extern pyro::booster_cmd_t *booster_cmd_ptr;

namespace pyro
{

void shoot_17mm_control_t::state_cali_forward_t::enter(owner *ctx)
{
    // --- 切回位置环, 目标为相对于偏移的一个小角度 ---
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::POSITION;
    ctx->_ctx.data.trig_pid_active    = true;
    // 目标角度 = 偏移 + CALI_FORWARD_TARGET_RAD (从死区正转一小段)
    ctx->_ctx.data.cali_target_rad    = ctx->_ctx.data.current_trig_rad + CALI_FORWARD_TARGET_RAD;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.cali_target_rad;
    ctx->_ctx.data.current_state      = data_ctx_t::state_e::CALI_FORWARD;
}

void shoot_17mm_control_t::state_cali_forward_t::execute(owner *ctx)
{
    // --- 紧急退出 ---
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // --- 计算当前角度误差 ---
    float err = ctx->_ctx.data.target_trig_rad - ctx->_ctx.data.current_trig_rad;

    // --- 校准完成判定 ---
    if (std::abs(err) < CALI_DONE_ANGLE_THRESHOLD)
    {
        ctx->_ctx.data.is_calibrated = true;

        // --- 根据目标状态恢复 ---
        switch (ctx->_ctx.data.target_state_after_cali)
        {
            case data_ctx_t::state_e::SINGLE_BULLET:
                this->request_switch(&ctx->_state_single_bullet);
                break;

            case data_ctx_t::state_e::CONTINUE_BULLET:
                this->request_switch(&ctx->_state_continue_bullet);
                break;

            default:
                this->request_switch(&ctx->_state_ready_shoot);
                break;
        }
    }
}

void shoot_17mm_control_t::state_cali_forward_t::exit(owner *ctx)
{
    // 校准完成, 清空积分
    ctx->_ctx.booster_cfg.pid.trig_pos_pid->clear();
    ctx->_ctx.booster_cfg.pid.trig_spd_pid->clear();

    // 重置目标角度为校准后的当前位置, 为后续单发提供准确起点
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad;

    // 首次校准完成时清除触发标志 (已进入目标状态)
    if (ctx->_ctx.data.jam_source_state == data_ctx_t::state_e::READY_SHOOT)
    {
        booster_cmd_ptr->single_shoot  = false;
        booster_cmd_ptr->continue_shoot = false;
    }
}

} // namespace pyro