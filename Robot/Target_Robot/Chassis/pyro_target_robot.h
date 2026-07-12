#ifndef __PYRO_TARGET_ROBOT_H__
#define __PYRO_TARGET_ROBOT_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_kin_rudomni.h"
#include "pyro_motor_base.h"
#include "target_robot_config.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct target_cmd_t : cmd_base_t
{
    float vx;               // 云台坐标系下的 X 轴速度 m/s (推前)
    float vy;               // 云台坐标系下的 Y 轴速度 m/s (推左)
    float wz;               // z轴角速度 rad/s (跟随模式下由follow_yaw_pid计算)
    float wz2;              // IMU锁头模式下 yaw 轴角速度增量 rad/s (摇杆ry)
    float target_imu_delta; // 跟随模式下IMU目标累积增量 rad (由摇杆rx累积, 加至target_insyaw)
    bool follow_yaw;        // 是否开启底盘跟随云台 (true=跟随, false=IMU锁头)

    target_cmd_t()
        : vx(0), vy(0), wz(0), wz2(0), target_imu_delta(0), follow_yaw(false)
    {
    }
};

// =========================================================
// 2. 依赖定义 (电机句柄 + PID句柄)
// =========================================================
struct target_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *wheel[4]{nullptr};     // 4个全向轮 (M3508)
        motor_base_t *steering[2]{nullptr};  // 2个舵轮电机 (GM6020)
        motor_base_t *yaw{nullptr};          // 1个yaw轴云台电机 (GM6020)
    };

    struct pid_deps_t
    {
        pid_t *wheel_pid[4]{nullptr};       // 轮子速度环PID
        pid_t *steering_pos_pid[2]{nullptr}; // 舵机位置环PID
        pid_t *steering_spd_pid[2]{nullptr}; // 舵机速度环PID
        pid_t *yaw_pos_pid{nullptr};         // yaw位置环PID (IMU锁头)
        pid_t *yaw_spd_pid{nullptr};         // yaw速度环PID
        pid_t *follow_yaw_pid{nullptr};      // 底盘跟随yaw的PID (current_yaw_rad→0→wz)
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};

    float steering_pos_offset[3]{}; // 舵机零点偏移[0]=舵机0,[1]=舵机1,[2]=yaw
};

// =========================================================
// 3. 运行时数据上下文
// =========================================================
struct target_data_ctx_t
{
    rudomni_kin_t::rudomni_states_t current_states{};
    rudomni_kin_t::rudomni_states_t target_states{};

    float current_steering_radps[2]{};   // 舵机当前角速度
    float current_yaw_rad{0};            // yaw电机当前位置 (带偏移)
    float current_yaw_radps{0};          // yaw电机当前角速度

    float out_steering_torque[2]{};      // 舵机输出扭矩
    float out_wheel_torque[4]{};         // 轮子输出扭矩

    float target_yaw_rads{0};            // yaw目标角速度
    float out_yaw_torque{};              // yaw输出扭矩
};

struct target_context_t
{
    target_deps_t::motor_deps_t motor;
    target_deps_t::pid_deps_t pid;
    target_data_ctx_t data;
    float steering_pos_offset[3]{};  // 舵机零点偏移
    target_cmd_t *cmd{};
};

// =========================================================
// 4. Yaw 数据结构 (公开，App层写入IMU数据)
// =========================================================
struct target_yaw_data_t
{
    float current_insyaw{0};    // 当前IMU yaw角
    float current_inspitch{0};  // 当前IMU pitch角
    float current_insroll{0};   // 当前IMU roll角
    float target_insyaw{0};     // 目标IMU yaw角
};

// =========================================================
// 5. ModuleParams 聚合类型
// =========================================================
struct target_module_params_t
{
    using CmdType    = target_cmd_t;
    using ModuleDeps = target_deps_t;
    using ModuleCtx  = target_context_t;
};

// =========================================================
// 6. 舵-全向混合底盘模块类
// =========================================================
class target_robot_t final
    : public module_base_t<target_robot_t, target_module_params_t>
{
    friend class module_base_t<target_robot_t, target_module_params_t>;

  public:
    target_robot_t(const target_robot_t &)            = delete;
    target_robot_t &operator=(const target_robot_t &) = delete;
    using data_ctx_t      = target_data_ctx_t;
    using target_context_t = pyro::target_context_t;
    using yaw_data_t      = target_yaw_data_t;

    /// 公开的yaw数据，App层可通过 ins_drv->get_rads_b() 填入IMU数据
    yaw_data_t _yaw_data;

  private:
    target_robot_t();
    ~target_robot_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    void _kinematics_solve();
    void _chassis_control();
    void _send_motor_command() const;
    static void yaw_angle_ch(float current_angle, float &target_angle);

    rudomni_kin_t *_kinematics{nullptr};

    // --- FSM 状态定义 ---
    using owner = target_robot_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // __PYRO_TARGET_ROBOT_H__
