#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"
#include "quad_config.h" // 新增：引入预置位宏定义 TRIGGER_OFFSET
#include <cmath>         // 新增：引入数学库以使用 std::round

namespace pyro
{
void quad_booster_t::fsm_active_t::state_stall_t::enter(owner *owner)
{


    if (&owner->_state_active._homing_state == owner->_state_active._last_state)
    {
        owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
        owner->_ctx.data.target_trig_radps = 0;
        // --- 修改点：寻找最近的预置位 ---
        // 1. 计算当前位置相对于基准预置位 TRIGGER_OFFSET 的偏差
        float delta = owner->_ctx.data.current_trig_rad - TRIGGER_OFFSET;

        // 由于过零点等原因，偏差可能超出单圈，先将其归一化（确保寻找的是物理上最近的位置）
        delta = quad_booster_t::_normalize_angle(delta);

        // 2. 拨叉有6个槽位，间距为 PI/3。用四舍五入(round)找出离当前位置最近的档位序号
        // 严格寻找 target 减小（正转）方向上的最近预置位
        float n = std::floor(delta / (PI / 3.0f));

        // 3. 计算出最近的绝对目标角度
        owner->_ctx.data.target_trig_rad = TRIGGER_OFFSET + n * (PI / 3.0f);
    }
    else if (&owner->_state_active._stall_state ==
             owner->_state_active._last_state)
    {
        owner->_ctx.data.target_trig_rad   = owner->_ctx.data.current_trig_rad;
        owner->_ctx.data.target_trig_radps = 0;
        // 什么都不做
    }
    else
    {
        owner->_ctx.data.target_trig_rad += PI / 3.0f; // 正常发弹时的堵转反转退弹逻辑
    }

    // 新增：保证目标值在 [-PI, PI] 合法范围内
    owner->_ctx.data.target_trig_rad = quad_booster_t::_normalize_angle(owner->_ctx.data.target_trig_rad);
}

void quad_booster_t::fsm_active_t::state_stall_t::execute(owner *owner)
{

    // 新增：计算最短路径的误差
    float error = owner->_ctx.data.current_trig_rad - owner->_ctx.data.target_trig_rad;
    error = quad_booster_t::_normalize_angle(error);

    // 回到合适角度后，切换回拨弹状态
    if (fabs(error) < 0.12f)
    {
        request_switch(&owner->_state_active._interim_state);
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_stall_t::exit(owner *owner)
{
}
} // namespace pyro