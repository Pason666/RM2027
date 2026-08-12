#ifndef __PYRO_PYRO_17MM_BOOSTER_H__
#define __PYRO_PYRO_17MM_BOOSTER_H__

#include "pyro_core_fsm.h"
#include "pyro_algo_pid.h"
#include "pyro_algo_common.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_17mm_config.h"
#include "pyro_heat_controller.h"

namespace pyro
{
struct booster_cmd_t : cmd_base_t
{
    bool is_fric_on;            // 摩擦轮是否开启
    bool single_shoot;          // 触发单发
    bool continue_shoot;        // 触发连发
    bool heat_control_on{true}; // 热量控制开关，false时跳过所有热量限制（调试用）
    bool fire_licence{};        // 发射许可，为false时拨弹盘绝对不允许转动

    uint16_t ammo_count{};        // 剩余发弹量（裁判系统反馈）
    uint16_t power_heat{};        // 当前热量（裁判系统反馈）
    uint16_t heat_limit{};        // 热量上限（裁判系统反馈）
    uint16_t cooling_rate{};      // 冷却速率（裁判系统反馈）
    float current_bullet_mps{};   // 当前弹速（裁判系统反馈）

    booster_cmd_t()
        : is_fric_on(false), single_shoot(false), continue_shoot(false)
    {
    }
};

struct booster_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *fric[2]{nullptr};
        motor_base_t *trigger{nullptr};
    };
    struct pid_cfg_t
    {
        pid_t *trig_pos_pid{nullptr};
        pid_t *trig_spd_pid{nullptr};
        pid_t *fric_pid[2]{nullptr};
        pid_t *bullet_speed_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
    float target_fric_speed;
};

class shoot_17mm_control_t final
    : public module_base_t<shoot_17mm_control_t, booster_cmd_t, booster_cfg_t>
{
    friend class module_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    shoot_17mm_control_t(const shoot_17mm_control_t &)            = delete;
    shoot_17mm_control_t &operator=(const shoot_17mm_control_t &) = delete;

  private:
    shoot_17mm_control_t();
    ~shoot_17mm_control_t() override    = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    static void _fric_control(shoot_17mm_control_t *ctx);
    static void _trig_control(shoot_17mm_control_t *ctx);
    static void _send_motor_command(booster_ctx_t *ctx);

    struct data_ctx_t
    {
        // 供状态机内部读取的状态变量
        uint16_t block_time     = 0;
        bool fric_pid_active    = true;
        bool trig_pid_active    = true;
        enum class trig_mode_e
        {
            SPEED,
            POSITION
        } trig_mode          = trig_mode_e::SPEED;
        bool is_first_update = true; // 计圈辅助变量
        float last_rotor_rad = 0.0f; // 计圈辅助变量
        float current_fric_radps[2]{};
        float current_trig_rad{};
        float current_trig_radps{};
        float target_fric_radps[2]{};
        float target_trig_radps{};
        float target_trig_rad{};
        float out_trig_radps{};
        float out_trig_torque{};
        float out_fric_torque[2]{};

        // --- 校准相关 ---
        bool is_calibrated          = false;    // 是否已完成拨弹盘校准 (单发需要)
        float trigger_offset        = 0.0f;     // 编码器零点偏移 (校准后确定)
        uint32_t block_start_tick   = 0;        // 堵转检测起始时刻 (0=未堵转)
        float cali_target_rad       = 0.0f;     // 校准正转目标角度

        // --- 状态枚举 (用于记录堵转来源) ---
        enum class state_e {
            STOP,
            READY_FRIC,
            READY_SHOOT,
            SINGLE_BULLET,
            CONTINUE_BULLET,
            DONE,
            CALI_REVERSE,
            CALI_FORWARD
        } current_state = state_e::STOP;
        state_e jam_source_state     = state_e::STOP;    // 堵转来源状态
        state_e target_state_after_cali = state_e::READY_SHOOT; // 校准完成后目标状态

        float fric_radps_error{}; // 供连发状态记录当前弹速误差
        bool continue_heat_limited{}; // 连发热量限制导致拨弹目标速度为0
        bool suppress_continue_recovery_cali{}; // 热量恢复连发阶段跳过校准

        // --- 弹速滑动窗口 ---
        static constexpr uint8_t BULLET_SPEED_WINDOW_SIZE = 8;
        float bullet_speed_buffer[BULLET_SPEED_WINDOW_SIZE]{}; // 弹速缓冲区
        uint8_t bullet_speed_index = 0;                         // 缓冲区索引
        uint8_t bullet_speed_count = 0;                         // 已记录弹速数量（用于启动阶段）

        // --- 热量控制器 ---
        HeatController heatController;
        float last_shot_trig_rad = 0.0f; // 上次发弹时的拨弹盘角度（物理发弹检测）
    };

    struct booster_ctx_t
    {
        booster_cfg_t booster_cfg;
        data_ctx_t data;
        booster_cmd_t *cmd{};
    };

    struct debug_ctx_t
    {
        float debug_rud_torque[4]{};
    };

    booster_ctx_t _ctx;
    debug_ctx_t debug_data;

    // ================== FSM 状态机定义 ==================
    using owner = shoot_17mm_control_t;

    struct state_stop_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_ready_fric_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_ready_shoot_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_single_bullet_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_continue_bullet_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_done_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    // --- 校准状态 ---
    struct state_cali_reverse_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_cali_forward_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };

    fsm_t<owner> _main_fsm;
    state_stop_t _state_stop;
    state_ready_fric_t _state_ready_fric;
    state_ready_shoot_t _state_ready_shoot;
    state_single_bullet_t _state_single_bullet;
    state_continue_bullet_t _state_continue_bullet;
    state_done_t _state_done;
    state_cali_reverse_t _state_cali_reverse;
    state_cali_forward_t _state_cali_forward;
};

} // namespace pyro

#endif // PYRO_PYRO_17MM_BOOSTER_H
