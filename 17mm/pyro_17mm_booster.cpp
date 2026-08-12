#include "pyro_17mm_booster.h"
#include <cmath>
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_core_def.h"
#include "pyro_sentry_gimbal.h"

float test_fric_speed[2]{0.0f, 0.0f};
float test_trig_pos = 0.0f;
float test_trig_torque = 0.0f;
float test_heat = 0.0f; // 热量调试变量

namespace pyro
{


// ================== 基础接口实现 ==================
shoot_17mm_control_t::shoot_17mm_control_t()
    : module_base_t("booster", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

status_t shoot_17mm_control_t::_init()
{
    _ctx.booster_cfg = _module_deps;
    return PYRO_OK;
}

void shoot_17mm_control_t::_update_feedback()
{
    _ctx.booster_cfg.motor.fric[0]->update_feedback();
    _ctx.booster_cfg.motor.fric[1]->update_feedback();
    _ctx.booster_cfg.motor.trigger->update_feedback();

    _ctx.data.current_fric_radps[0] =
        _ctx.booster_cfg.motor.fric[0]->get_current_rotate();
    _ctx.data.current_fric_radps[1] =
        _ctx.booster_cfg.motor.fric[1]->get_current_rotate();
    test_fric_speed[0] = _ctx.data.current_fric_radps[0] + 625.67f;
    test_fric_speed[1] = -_ctx.data.current_fric_radps[1] + 625.67f;

    float current_rotor_rad =
        _ctx.booster_cfg.motor.trigger->get_current_position();

    if (_ctx.data.is_first_update)
    {
        // 第一次开机，初始化基准角度
        _ctx.data.last_rotor_rad   = current_rotor_rad;
        _ctx.data.current_trig_rad = current_rotor_rad / TRIGGER_GEAR_RATIO;
        _ctx.data.is_first_update  = false;
    }
    else
    {
        // 1. 计算转子转动的极短增量
        float delta_rad = current_rotor_rad - _ctx.data.last_rotor_rad;

        // 2. 处理跨圈突变边界 (0 <-> 2PI 或 -PI <-> PI)
        delta_rad       = wrap_pi(delta_rad);

        // 3. 将物理增量除以减速比，累加到连续的拨弹盘世界角度中
        _ctx.data.current_trig_rad += delta_rad / TRIGGER_GEAR_RATIO;

        // 更新历史值
        _ctx.data.last_rotor_rad = current_rotor_rad;
    }
    test_trig_pos = _ctx.data.current_trig_rad;

    _ctx.data.current_trig_radps =
        _ctx.booster_cfg.motor.trigger->get_current_rotate() /
        TRIGGER_GEAR_RATIO;
}

void shoot_17mm_control_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;
    if (!_ctx.cmd->is_fric_on)
        _main_fsm.change_state(&_state_stop);

    uint32_t nowMs = xTaskGetTickCount();

    // 热量控制器: 裁判系统同步
    _ctx.data.heatController.syncWithReferee(
        _ctx.cmd->power_heat, _ctx.cmd->heat_limit,
        _ctx.cmd->cooling_rate, nowMs);

    // 热量控制器: 本地冷却推演 (dt ≈ 1ms 周期)
    _ctx.data.heatController.tickCooling(0.001f);

    // 物理发弹检测: 拨弹盘角度跨越 PI/4 → 注册一发
    float delta = _ctx.data.current_trig_rad - _ctx.data.last_shot_trig_rad;
    if (delta >= PI / 4.0f)
    {
        _ctx.data.heatController.recordBulletShot(nowMs);
        _ctx.data.last_shot_trig_rad += PI / 4.0f;
    }

    test_heat = _ctx.data.heatController.getLocalHeat();

    _main_fsm.execute(this);

    if (_ctx.cmd->heat_control_on)
        _ctx.cmd->fire_licence = _ctx.data.heatController.canShootSingle();
    else
        _ctx.cmd->fire_licence = true;

    _fric_control(this);
    _trig_control(this);
    _send_motor_command(&_ctx);
}

void shoot_17mm_control_t::_fric_control(shoot_17mm_control_t *ctx)
{
    for (int i = 0; i < 2; i++)
    {
        if (ctx->_ctx.data.fric_pid_active)
        {
            ctx->_ctx.data.out_fric_torque[i] =
                ctx->_ctx.booster_cfg.pid.fric_pid[i]->calculate(
                    ctx->_ctx.data.target_fric_radps[i],
                    ctx->_ctx.data.current_fric_radps[i]);
        }
        else
            ctx->_ctx.data.out_fric_torque[i] = 0.0f;
    }
}

void shoot_17mm_control_t::_trig_control(shoot_17mm_control_t *ctx)
{
    if (ctx->_ctx.data.trig_mode == data_ctx_t::trig_mode_e::POSITION)
    {
        // 1. 位置环：根据目标角度算出目标角速度
        ctx->_ctx.data.target_trig_radps =
            ctx->_ctx.booster_cfg.pid.trig_pos_pid->calculate(
                ctx->_ctx.data.target_trig_rad,
                ctx->_ctx.data.current_trig_rad);
    }
    // 2. 速度环：根据目标角速度算出扭矩 (无论是直接给定的还是位置环算出的)
    ctx->_ctx.data.out_trig_torque =
        ctx->_ctx.booster_cfg.pid.trig_spd_pid->calculate(
            ctx->_ctx.data.target_trig_radps,
            ctx->_ctx.data.current_trig_radps);

    if (!ctx->_ctx.data.trig_pid_active || !ctx->_ctx.cmd->fire_licence)
    {
        ctx->_ctx.data.out_trig_torque = 0.0f;
    }
}

void shoot_17mm_control_t::_send_motor_command(booster_ctx_t *ctx)
{
    ctx->booster_cfg.motor.fric[0]->send_torque(ctx->data.out_fric_torque[0]);
    ctx->booster_cfg.motor.fric[1]->send_torque(ctx->data.out_fric_torque[1]);
    ctx->booster_cfg.motor.trigger->send_torque(ctx->data.out_trig_torque);
    test_trig_torque = ctx->data.out_trig_torque;
}

} // namespace pyro