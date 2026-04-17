#ifndef __PYRO_HYBRID_CHASSIS_H__
#define __PYRO_HYBRID_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_hybrid.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h" // 新增 IMU 依赖
#include "hybrid_config.h"
#include "pyro_power_control_drv.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct hybrid_cmd_t : cmd_base_t
{
    float vx;          // 云台坐标系下的 X 轴速度 m/s (推前)
    float vy;          // 云台坐标系下的 Y 轴速度 m/s (推左)
    float wz;          // z轴角速度 rad/s (通常跟随模式下该值为0，除非做小陀螺)
    float delta_pitch; // 腿部目标位置相对于当前的增量 rad
    float delta_yaw;
    bool track_en;    // 是否启用履带 (true: 履带 + 麦轮混合驱动, false: 仅麦轮)
    bool leg_retract; // 是否进入腿部收回状态 (仅在 track_en=true 时有效)

    hybrid_cmd_t()
        : vx(0), vy(0), wz(0), delta_pitch(0), delta_yaw(0), track_en(false),
          leg_retract(false)
    {
    }
};

struct hybrid_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *mecanum[4]{nullptr};
        motor_base_t *track[2]{nullptr};
        motor_base_t *leg[2]{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象
    struct pid_deps_t
    {
        pid_t *mecanum_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
        pid_t *track_pid[2]{nullptr};
        pid_t *pitch_pid{nullptr};
        pid_t *roll_pid{nullptr};

        pid_t *leg_pos_pid[2]{nullptr}; // 腿部位置控制 PID
        pid_t *leg_vel_pid[2]{nullptr}; // 腿部速度控制 PID
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};

// =========================================================
// 2. 混合底盘类
// =========================================================
class hybrid_chassis_t final
    : public module_base_t<hybrid_chassis_t, hybrid_cmd_t, hybrid_deps_t>
{
    friend class module_base_t<hybrid_chassis_t, hybrid_cmd_t, hybrid_deps_t>;

    struct motor_deps_t;
    struct pid_deps_t;
    struct data_ctx_t;
    struct hybrid_context_t;

  public:
    hybrid_chassis_t(const hybrid_chassis_t &)            = delete;
    hybrid_chassis_t &operator=(const hybrid_chassis_t &) = delete;

  private:
    hybrid_chassis_t();
    ~hybrid_chassis_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    static void _power_control_init();
    void _kinematics_solve();
    void _power_control();
    void _mecanum_control();
    void _track_control();
    void _leg_vmc();
    void _leg_length_control();
    void _send_motor_command() const;
    void _communicate_gimbal() const;
    hybrid_kin_t *_kinematics{nullptr};

    // 运行时数据
    struct data_ctx_t
    {
        bool wheel_online[4]{};
        float current_wheel_rpm[4]{};
        float current_track_rpm[2]{};
        float current_leg_rad[2]{};
        float current_leg_radps[2]{};

        // IMU 姿态反馈
        float current_pitch_rad{0};
        float current_roll_rad{0};
        float current_yaw_rad{0};
        float target_pitch_rad{0};
        float target_yaw_rad{0};

        // 逆解算速度
        float real_vx{0};
        float real_vy{0};
        float real_wz{0};

        // 测距模块反馈
        uint16_t distance_mm{0};

        // YAW 电机差值反馈（用于底盘跟随云台）
        float current_yaw_error{0};
        float target_wheel_rpm[4]{};
        float target_track_rpm[2]{};
        float target_leg_rad[2]{};
        float target_leg_radps[2]{};

        // 输出
        float out_mecanum_torque[4]{};
        float out_track_torque[2]{};
        float out_leg_torque[2]{};
    };

    struct hybrid_context_t
    {
        hybrid_deps_t::motor_deps_t motor;
        hybrid_deps_t::pid_deps_t pid;
        data_ctx_t data;
        power_control_drv_t::motor_data_t power_motor_data[4]{};
        hybrid_cmd_t *cmd{};
    };

    // 总 Context
    hybrid_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = hybrid_chassis_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t final : public fsm_t<owner>
    {
        struct cruising_state_t final : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };
        struct climbing_fsm_t final : public fsm_t<owner>
        {
            struct track_climbing_state_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            struct leg_retraction_state_t final : public state_t<owner>
            {
                void enter(owner *owner) override;
                void execute(owner *owner) override;
                void exit(owner *owner) override;
            };

            void on_enter(owner *owner) override;
            void on_execute(owner *owner) override;
            void on_exit(owner *owner) override;

          private:
            track_climbing_state_t track_climbing_state;
            leg_retraction_state_t leg_retraction_state;

            // --- 新增：施密特触发器及自动收腿状态变量 ---
            float _filtered_distance{0.0f};         // 滤波后的距离
            bool  _is_guide_wheel_suspended{false}; // 施密特高位状态标志
            bool  _auto_retract_flag{false};        // 自动收腿触发标志
        };

        // FSM Hooks
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        cruising_state_t cruising_state;
        climbing_fsm_t climbing_fsm;
    };

    // 状态实例
    state_passive_t _state_passive;
    fsm_active_t _state_active;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif