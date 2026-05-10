#ifndef __PYRO_MEC_CHASSIS_H__
#define __PYRO_MEC_CHASSIS_H__

#include "pyro_algo_pid.h"
#include "pyro_kin_mec.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_supercap_drv.h"
#include "pyro_power_control_drv.h"
#include "pyro_powermeter.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct mec_cmd_t final : public cmd_base_t
{
    float vx; // 云台坐标系下的 X 轴速度 m/s (推前)
    float vy; // 云台坐标系下的 Y 轴速度 m/s (推左)
    float wz; // z轴角速度 rad/s (通常跟随模式下该值为0，除非做小陀螺)

    mec_cmd_t() : vx(0), vy(0), wz(0)
    {
    }
};

struct mec_cfg_t
{
};

// =========================================================
// 2. 麦轮底盘类
// =========================================================
class mec_chassis_t final : public module_base_t<mec_chassis_t, mec_cmd_t,mec_cfg_t>
{
    friend class module_base_t<mec_chassis_t, mec_cmd_t,mec_cfg_t>;
    friend class jcom_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct mec_context_t;

  public:
    mec_chassis_t(const mec_chassis_t &)            = delete;
    mec_chassis_t &operator=(const mec_chassis_t &) = delete;

  private:
    mec_chassis_t();
    ~mec_chassis_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    static void _power_control_init();
    void _power_control();
    void _kinematics_solve();
    void _chassis_control(mec_context_t *ctx);
    static void _send_motor_command(mec_context_t *ctx);
    void _send_supercap_command() const;

    // --- 成员变量 ---
    mecanum_kin_t *_kinematics{nullptr};

    struct motor_ctx_t
    {
        motor_base_t *wheels[4]{nullptr}; // FL, FR, BL, BR
        motor_base_t *yaw_motor{nullptr};
    };

    struct pid_ctx_t
    {
        pid_t *follow_pid{nullptr};
        pid_t *wheel_pid[4]{nullptr};
    };

    struct data_ctx_t
    {
        float current_yaw_error{0}; // 归一化后的偏航误差 (-PI ~ PI)

        bool wheel_online[4]{};
        float current_wheel_rpm[4]{};
        float target_wheel_rpm[4]{};
        float out_wheel_torque[4]{};
    };

    struct mec_context_t
    {
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        mec_cmd_t *cmd{};
        supercap_drv_t::chassis_cmd_t supercap_cmd{};
        supercap_drv_t::cap_feedback_t cap_feedback{};
        powermeter_drv_t *powermeter{nullptr};
        powermeter_data powermeter_feedback{};
        power_control_drv_t::motor_data_t power_motor_data[4]{};
    };
    float buffer_engy;

    mec_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = mec_chassis_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct state_active_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 物理参数常量
    // =====================================================
    static constexpr float WHEEL_RADIUS = 0.076f; // m
    static constexpr float WHEELBASE    = 0.380f; // m
    static constexpr float TRACK_WIDTH  = 0.380f; // m

    static constexpr float YAW_OFFSET_RAD = 0.796136022f; // 云台归中时的机械偏移量
};

} // namespace pyro
#endif