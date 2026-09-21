#include "pyro_large_launcher.h"
#include "pyro_algo_common.h"
#include "pyro_bsp_uart.h"
#include "pyro_board_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_referee.h"
#include "large_launcher_config.h"

#include <cmath>
#include <algorithm>

namespace pyro
{

tri_booster_t::tri_booster_t() : module_base_t("tri_booster")
{
}

status_t tri_booster_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;
    _ctx.pid.ball_speed_pid = new pid_t(0.4f, 0.002f, 0.005f, 0.01f, 2.0f);

    return PYRO_OK;
}

float tri_booster_t::_normalize_angle(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

bool tri_booster_t::_is_trigger_located(float trigger_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const float nearest_slot_delta =
        delta - std::round(delta / TRIGGER_SLOT_RAD) * TRIGGER_SLOT_RAD;
    return std::fabs(nearest_slot_delta) < TRIGGER_LOCATED_THRESHOLD_RAD;
}

float tri_booster_t::_get_next_trigger_preset(float trigger_rad,
                                               float min_advance_rad)
{
    constexpr float TRIGGER_SLOT_RAD = PI / 3.0f;
    const float delta = _normalize_angle(trigger_rad - TRIGGER_OFFSET);
    const bool feed_positive = TRIGGER_FEED_DIR > 0.0f;
    float preset_index = feed_positive
                             ? std::ceil(delta / TRIGGER_SLOT_RAD)
                             : std::floor(delta / TRIGGER_SLOT_RAD);
    const float advance = feed_positive
                              ? preset_index * TRIGGER_SLOT_RAD - delta
                              : delta - preset_index * TRIGGER_SLOT_RAD;

    if (advance <= min_advance_rad)
    {
        preset_index += feed_positive ? 1.0f : -1.0f;
    }

    return _normalize_angle(TRIGGER_OFFSET + preset_index * TRIGGER_SLOT_RAD);
}

void tri_booster_t::_update_feedback()
{
    // 更新三个摩擦轮的反馈
    for (int i = 0; i < 3; i++)
    {
        _ctx.motor.fric_wheels[i]->update_feedback();
        _ctx.data.current_fric_torque[i] =
            _ctx.motor.fric_wheels[i]->get_current_torque();
    }

    // 计算摩擦轮线速度 (m/s)
    _ctx.data.current_fric_mps[0] =
        _ctx.motor.fric_wheels[0]->get_current_rotate() * FRIC1_RADIUS;
    _ctx.data.current_fric_mps[1] =
        _ctx.motor.fric_wheels[1]->get_current_rotate() * FRIC2_RADIUS;
    _ctx.data.current_fric_mps[2] =
        _ctx.motor.fric_wheels[2]->get_current_rotate() * FRIC3_RADIUS;

    for (int i = 0; i < 3; i++)
    {
        _ctx.data.abs_current_fric_mps[i] = abs(_ctx.data.current_fric_mps[i]);
    }

    // 更新拨弹盘反馈
    _ctx.motor.trigger_wheel->update_feedback();
    _ctx.data.current_trig_radps = _ctx.motor.trigger_wheel->get_current_rotate();
    _ctx.data.current_trig_torque = _ctx.motor.trigger_wheel->get_current_torque();
    _ctx.data.current_trig_rad = _ctx.motor.trigger_wheel->get_current_position();
    _ctx.data.trigger_located = _is_trigger_located(_ctx.data.current_trig_rad);
}

void tri_booster_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
        _main_fsm.change_state(&_state_active);
    else
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}

void tri_booster_t::_speed_control()
{
    static uint16_t last_launching_num = 0;
    auto &referee = referee_drv_t::get_instance();

    if (!referee->is_online()) return;

    const auto &referee_data = referee->get_data();

    auto &shoot_data = _ctx.shoot_data;
    _ctx.data.target_shoot_speed = shoot_data.target_speed;

    uint16_t current_launching_num = referee_data.shoot_data.launching_frequency;
    float shoot_speed = referee_data.shoot_data.bullet_speed;

    if (current_launching_num == last_launching_num)
    {
        return;
    }
    last_launching_num = current_launching_num;

    if (shoot_speed - _ctx.data.target_shoot_speed > 0.3f)
    {
        float err = shoot_speed - _ctx.data.target_shoot_speed;
        shoot_data.fric_mps -= err;
        return;
    }

    if (fabs(_ctx.data.target_shoot_speed - shoot_speed) > 2.0f)
    {
        return;
    }

    for (int i = 7; i > 0; --i)
    {
        shoot_data.real_ball_speed[i] = shoot_data.real_ball_speed[i - 1];
    }
    shoot_data.real_ball_speed[0] = shoot_speed;

    constexpr float real_speed_weight = 1.0f / 8.0f;
    shoot_data.avg_real_ball_speed = 0.0f;
    for (float speed : shoot_data.real_ball_speed)
    {
        shoot_data.avg_real_ball_speed += real_speed_weight * speed;
    }

    shoot_data.ball_speed[2] = shoot_data.ball_speed[1];
    shoot_data.ball_speed[1] = shoot_data.ball_speed[0];
    shoot_data.ball_speed[0] = shoot_speed;

    for (float &i : shoot_data.ball_speed)
    {
        if (i == 0.0f)
            i = shoot_data.ball_speed[0];
    }

    for (float &i : shoot_data.real_ball_speed)
    {
        if (i == 0.0f)
            i = shoot_data.real_ball_speed[0];
    }

    constexpr float outlier_threshold = 0.1f;
    const bool real_speed_stable =
        std::abs(shoot_data.avg_real_ball_speed - shoot_data.target_speed) <
        outlier_threshold;

    if (real_speed_stable &&
        std::abs(shoot_data.ball_speed[0] - shoot_data.target_speed) >
            outlier_threshold)
    {
        shoot_data.ball_speed[0] =
            0.7f * shoot_data.ball_speed[1] + 0.3f * shoot_data.ball_speed[2];
    }

    constexpr float w0 = 0.65f;
    constexpr float w1 = 0.25f;
    constexpr float w2 = 0.10f;

    shoot_data.avg_ball_speed = w0 * shoot_data.ball_speed[0] +
                                w1 * shoot_data.ball_speed[1] +
                                w2 * shoot_data.ball_speed[2];

    float e0 = shoot_data.ball_speed[0] - shoot_data.target_speed;
    float e1 = shoot_data.ball_speed[1] - shoot_data.target_speed;
    float e2 = shoot_data.ball_speed[2] - shoot_data.target_speed;

    float signed_weighted_mse = (w0 * e0 * std::abs(e0)) +
                                (w1 * e1 * std::abs(e1)) +
                                (w2 * e2 * std::abs(e2));

    float speed_increment =
        _ctx.pid.ball_speed_pid->calculate(0.0f, signed_weighted_mse);

    shoot_data.fric_mps += speed_increment;
    shoot_data.fric_mps = std::clamp(shoot_data.fric_mps, 9.0f, 17.0f);
}

void tri_booster_t::_launch_delay_calculate()
{
    auto &shoot_data = _ctx.shoot_data;

    _ctx.data.fresh_timer++;

    if (shoot_data.fric_mps - std::abs(_ctx.data.current_fric_mps[0]) > 0.8f &&
        shoot_data.fric_mps - std::abs(_ctx.data.current_fric_mps[1]) > 0.8f &&
        std::abs(_ctx.data.current_fric_torque[0]) > 3.0f &&
        std::abs(_ctx.data.current_fric_torque[1]) > 3.0f &&
        _ctx.data.fresh_timer > 220)
    {
        _ctx.data.launch_delay_timer[2] = _ctx.data.launch_delay_timer[1];
        _ctx.data.launch_delay_timer[1] = _ctx.data.launch_delay_timer[0];
        _ctx.data.launch_delay_timer[0] =
            (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer > 200.0f)
                ? _ctx.data.avg_launch_delay
                : (dwt_drv_t::get_timeline_ms() - _ctx.data.signal_timer + 20.0f);

        _ctx.data.avg_launch_delay = 0.7f * _ctx.data.launch_delay_timer[0] +
                                     0.2f * _ctx.data.launch_delay_timer[1] +
                                     0.1f * _ctx.data.launch_delay_timer[2];
        _ctx.data.fresh_timer = 0;
        _ctx.data.fire_count++;
    }
}

void tri_booster_t::_reset_active_shoot_data()
{
    _ctx.shoot_data.reset();
}

void tri_booster_t::_fric_control()
{
    for (int i = 0; i < 3; i++)
    {
        _ctx.data.out_fric_torque[i] = _ctx.pid.fric_pid[i]->calculate(
            _ctx.data.target_fric_mps[i], _ctx.data.current_fric_mps[i]);
    }
}

void tri_booster_t::_trigger_position_control()
{
    float error = _ctx.data.target_trig_rad - _ctx.data.current_trig_rad;
    error       = _normalize_angle(error);

    _ctx.data.target_trig_radps = _ctx.pid.trigger_pos_pid->calculate(error, 0.0f);

    static float ff_torque = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 1.0f;
    constexpr float TRIG_FF_TORQUE = 0.505f;

    const float feed_speed = _ctx.data.target_trig_radps * TRIGGER_FEED_DIR;
    if (feed_speed > TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }
    else if (feed_speed < 0.0f)
    {
        ff_torque = 0.0f;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) + ff_torque;

    _ctx.data.out_trig_torque = std::clamp(_ctx.data.out_trig_torque, -7.0f, 7.0f);
}

void tri_booster_t::_trigger_speed_control()
{
    float ff_torque                        = 0.0f;
    constexpr float TRIG_FF_SPEED_DEADBAND = 0.5f;
    constexpr float TRIG_FF_TORQUE         = 0.505f;

    if (_ctx.data.target_trig_radps * TRIGGER_FEED_DIR >
        TRIG_FF_SPEED_DEADBAND)
    {
        ff_torque = TRIGGER_FEED_DIR * TRIG_FF_TORQUE;
    }

    _ctx.data.out_trig_torque =
        _ctx.pid.trigger_spd_pid->calculate(_ctx.data.target_trig_radps,
                                            _ctx.data.current_trig_radps) + ff_torque;
}

void tri_booster_t::_send_fric_command() const
{
    for (int i = 0; i < 3; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(
            _ctx.data.out_fric_torque[i] +
            0.08f * _ctx.data.current_fric_torque[i]);
    }
}

void tri_booster_t::_send_raw_fric_command() const
{
    for (int i = 0; i < 3; i++)
    {
        _ctx.motor.fric_wheels[i]->send_torque(_ctx.data.out_fric_torque[i]);
    }
}

void tri_booster_t::_send_trigger_command() const
{
    _ctx.motor.trigger_wheel->send_torque(_ctx.data.out_trig_torque);
}

} // namespace pyro
