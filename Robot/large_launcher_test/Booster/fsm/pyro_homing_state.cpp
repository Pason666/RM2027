#include "pyro_dwt_drv.h"
#include "pyro_large_launcher.h"
#include "large_launcher_config.h"

namespace pyro
{
void tri_booster_t::fsm_active_t::state_homing_t::enter(owner *owner)
{
    owner->_ctx.data.homing_jam_start_tick = 0;
    owner->_ctx.data.homing_jam_detected = false;
}

void tri_booster_t::fsm_active_t::state_homing_t::execute(owner *owner)
{
    constexpr float HOMING_SPEED = -6.0f;  // 反转速度
    constexpr float JAM_SPEED_THRESHOLD = 1.0f;  // 卡住速度阈值（rad/s）
    constexpr uint32_t JAM_DETECT_TIME_MS = 200;  // 卡住检测时间（ms）
    constexpr float FORWARD_POSITION_THRESHOLD = 0.05f;  // 正转到位误差（rad）

    uint32_t now_tick = xTaskGetTickCount();

    // 阶段1：反转堵转检测
    if (!owner->_ctx.data.homing_jam_detected)
    {
        // 拨弹盘反转
        owner->_ctx.data.target_trig_radps = HOMING_SPEED * TRIGGER_FEED_DIR;
        owner->_trigger_speed_control();
        owner->_send_trigger_command();

        // 卡住检测：速度低于阈值
        if (std::abs(owner->_ctx.data.current_trig_radps) < JAM_SPEED_THRESHOLD)
        {
            if (owner->_ctx.data.homing_jam_start_tick == 0)
            {
                owner->_ctx.data.homing_jam_start_tick = now_tick;
            }
            else if (now_tick - owner->_ctx.data.homing_jam_start_tick >=
                     pdMS_TO_TICKS(JAM_DETECT_TIME_MS))
            {
                // 卡住确认 → 进入正转阶段
                owner->_ctx.data.homing_jam_detected = true;
                owner->_ctx.data.homing_forward_target =
                    owner->_ctx.data.current_trig_rad + HOMING_FORWARD_OFFSET;
                owner->_ctx.data.target_trig_rad = owner->_ctx.data.homing_forward_target;
            }
        }
        else
        {
            // 速度正常，重置计时器
            owner->_ctx.data.homing_jam_start_tick = 0;
        }
    }
    // 阶段2：正转偏移并记录零点
    else
    {
        // 位置控制正转
        owner->_trigger_position_control();
        owner->_send_trigger_command();

        // 检查是否到位
        float position_error = std::abs(owner->_ctx.data.current_trig_rad -
                                        owner->_ctx.data.homing_forward_target);
        if (position_error < FORWARD_POSITION_THRESHOLD)
        {
            // 记录当前位置为零点
            owner->_ctx.data.trigger_zero_position = owner->_ctx.data.current_trig_rad;

            // 进入interim状态
            request_switch(&owner->_state_active._interim_state);
            return;
        }
    }
}

void tri_booster_t::fsm_active_t::state_homing_t::exit(owner *owner)
{
    owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
    owner->_ctx.data.target_trig_radps = 0.0f;
}
} // namespace pyro
