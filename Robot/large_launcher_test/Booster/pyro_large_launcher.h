#ifndef __PYRO_LARGE_LAUNCHER_H__
#define __PYRO_LARGE_LAUNCHER_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_module_base.h"
#include <cmath>

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct tri_booster_cmd_t final : public cmd_base_t
{
    bool fric_on;       // 摩擦轮开启
    uint8_t reset_count;
    uint8_t shoot_data_reset_count;

    uint8_t fire_count; // 拨弹计数器

    float trig_target_spd; // 拨弹盘目标速度

    tri_booster_cmd_t()
        : fric_on(false), reset_count(0),
          shoot_data_reset_count(0), fire_count(0), trig_target_spd(0.0f)
    {
    }
};

struct tri_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *fric_wheels[3]{nullptr};  // 三个摩擦轮
        motor_base_t *trigger_wheel{nullptr};   // DM4310拨弹盘
    };

    struct pid_deps_t
    {
        pid_t *fric_pid[3]{nullptr};            // 三个摩擦轮PID
        pid_t *trigger_pos_pid{nullptr};
        pid_t *trigger_spd_pid{nullptr};
        pid_t *ball_speed_pid{nullptr};
    };

    motor_deps_t motor_deps;
    pid_deps_t pid_deps;
};

// =========================================================
// 2. 三轮发射机构类
// =========================================================
struct tri_booster_data_ctx_t
{
    uint8_t internal_fire_count{0};
    bool trigger_located{false};
    bool ready_state_flag{false};

    bool fric_err{};
    uint8_t internal_reset_count{0};
    uint8_t internal_shoot_data_reset_count{0};

    float launch_delay_timer[3]{};
    float signal_timer{0};
    float avg_launch_delay{0};
    uint32_t fresh_timer{0};
    uint8_t fire_count{0};

    float abs_current_fric_mps[3]{};
    float current_fric_mps[3]{};
    float current_trig_radps{0};
    float current_trig_torque{0};
    float current_trig_rad{0};

    float target_fric_mps[3]{};
    float target_trig_rad{0};
    float target_trig_radps{0};
    float target_shoot_speed{0};

    float current_fric_torque[3]{};
    float out_fric_torque[3]{};
    float out_trig_torque{0};
};

struct tri_booster_shoot_data_t
{
    const float target_speed{};
    float fric_mps{};  // 三个摩擦轮统一速度
    const float initial_fric_mps = fric_mps;
    float ball_speed[3]{};
    float avg_ball_speed = target_speed;
    float real_ball_speed[8]{};
    float avg_real_ball_speed = target_speed;

    void reset()
    {
        fric_mps = initial_fric_mps;
        for (float &speed : ball_speed)
        {
            speed = 0.0f;
        }
        for (float &speed : real_ball_speed)
        {
            speed = 0.0f;
        }
        avg_ball_speed      = target_speed;
        avg_real_ball_speed = target_speed;
    }
};

struct tri_booster_context_t
{
    tri_deps_t::motor_deps_t motor;
    tri_deps_t::pid_deps_t pid;
    tri_booster_data_ctx_t data;
    tri_booster_shoot_data_t shoot_data{16.3f, 15.6f};  // target_speed, fric_mps
    tri_booster_cmd_t *cmd{};
};

struct tri_booster_module_params_t
{
    using CmdType    = tri_booster_cmd_t;
    using ModuleDeps = tri_deps_t;
    using ModuleCtx  = tri_booster_context_t;
};

class tri_booster_t final
    : public module_base_t<tri_booster_t, tri_booster_module_params_t>
{
    friend class module_base_t<tri_booster_t, tri_booster_module_params_t>;
    friend class jcom_drv_t;

  public:
    tri_booster_t(const tri_booster_t &)            = delete;
    tri_booster_t &operator=(const tri_booster_t &) = delete;
    using data_ctx_t    = tri_booster_data_ctx_t;
    using shoot_data_t  = tri_booster_shoot_data_t;
    using booster_ctx_t = tri_booster_context_t;

  private:
    tri_booster_t();
    ~tri_booster_t() override = default;

    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    void _speed_control();
    void _fric_control();
    void _trigger_position_control();
    void _trigger_speed_control();
    void _send_fric_command() const;
    void _send_raw_fric_command() const;
    void _send_trigger_command() const;
    void _launch_delay_calculate();
    void _reset_active_shoot_data();

    static float _normalize_angle(float angle);
    static bool _is_trigger_located(float trigger_rad);
    static float _get_next_trigger_preset(float trigger_rad,
                                          float min_advance_rad);

    using owner = tri_booster_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;

      private:
        bool _trigger_stopped{false};
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct state_homing_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _homing_turnback_start_time{0.0f};
        };
        struct state_interim_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _ready_wait_start_time{0.0f};
            float _fric_ready_start_time{0.0f};
        };
        struct state_ready_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;

          private:
            float _fric_unready_start_time{0.0f};
        };
        struct state_busy_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct state_stall_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_homing_t _homing_state;
        state_interim_t _interim_state;
        state_ready_t _ready_state;
        state_busy_t _busy_state;
        state_stall_t _stall_state;
    };

    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro
#endif
