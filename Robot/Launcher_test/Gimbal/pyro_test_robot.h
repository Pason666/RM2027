#ifndef __PYRO_TEST_ROBOT_H__
#define __PYRO_TEST_ROBOT_H__

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "test_robot_config.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct test_robot_cmd_t final : public cmd_base_t
{
    float yaw_rate;          // Yaw轴目标角速度 rad/s (来自遥控器左摇杆X)
    float pitch_rate;        // Pitch轴目标角速度 rad/s (来自遥控器左摇杆Y)
    float friction_speed;    // 摩擦轮目标转速 rad/s
    float feeder_speed;      // 拨弹盘连发目标转速 rad/s (进入连发时设定)
    bool  feeder_trigger;    // 拨弹盘触发信号 (上升沿触发单发)
    bool  fire_enable;       // 连发模式使能

    test_robot_cmd_t()
        : yaw_rate(0), pitch_rate(0),
          friction_speed(TEST_FRICTION_DEFAULT_SPEED),
          feeder_speed(TRIGGER_CONTINUOUS_RADPS),
          feeder_trigger(false), fire_enable(false)
    {
    }
};

// =========================================================
// 2. 依赖定义 (电机句柄 + PID句柄)
// =========================================================
struct test_robot_deps_t
{
    struct motor_deps_t
    {
        motor_base_t *yaw{nullptr};             // Yaw轴  GM6020 (CAN1, ID=1)
        motor_base_t *pitch{nullptr};           // Pitch轴 DM4310 (CAN1)
        motor_base_t *friction[2]{nullptr};     // 摩擦轮×2 M3508 (CAN2, ID=1,2)
        motor_base_t *feeder{nullptr};          // 拨弹盘 M2006 (CAN2, ID=3)
    };

    struct pid_deps_t
    {
        // Yaw轴: 位置环 + 速度环
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};

        // Pitch轴: 位置环 + 速度环
        pid_t *pitch_pos_pid{nullptr};
        pid_t *pitch_spd_pid{nullptr};

        // 摩擦轮: 速度环 ×2
        pid_t *friction_spd_pid[2]{nullptr};

        // 拨弹盘: 位置环 + 速度环
        pid_t *feeder_pos_pid{nullptr};
        pid_t *feeder_spd_pid{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t   pid_deps{};

    float yaw_pos_offset{0};    // Yaw轴零点偏移 (rad)
    float pitch_pos_offset{0};  // Pitch轴零点偏移 (rad)
};

// =========================================================
// 3. 运行时数据上下文
// =========================================================
struct test_robot_data_ctx_t
{
    // --- 云台 ---
    float current_yaw_rad{0};
    float current_yaw_radps{0};
    float target_yaw_rad{0};
    float target_yaw_radps{0};
    float out_yaw_torque{0};

    float current_pitch_rad{0};
    float current_pitch_radps{0};
    float target_pitch_rad{0};
    float target_pitch_radps{0};
    float out_pitch_torque{0};

    // --- 摩擦轮 ---
    float current_friction_radps[2]{};
    float out_friction_torque[2]{};
    bool  fric_pid_active{true};    // 停机时: PID刹车 → 停转后切零力矩

    // --- 拨弹盘 (多圈连续角度) ---
    float current_feeder_rad{0};        // 连续的拨弹盘世界角度 (累积, 无±PI缠绕)
    float current_feeder_radps{0};      // 拨弹盘角速度 (已除以减速比)
    float target_feeder_rad{0};         // 目标位置 (位置环模式)
    float target_feeder_radps{0};       // 目标速度 (速度环模式)
    float out_feeder_torque{0};

    // --- 多圈角度追踪 ---
    bool  feeder_first_update{true};    // 首次初始化标志
    float feeder_last_rotor_rad{0};     // 上一次转子编码器读数

    // --- 发射计数 ---
    uint32_t fire_count{0};
    float    last_shot_feeder_rad{0};   // 上次发弹时的拨弹盘角度 (物理发弹检测)

    // --- 拨弹盘子状态 ---
    enum class trig_state_e : uint8_t
    {
        READY,         // 就绪: 位置环锁位
        SINGLE,        // 单发: 位置环推进 PI/4
        CONTINUE,      // 连发: 速度环恒转速
        CALI_REVERSE,  // 校准反转: 高速反转找机械死区
        CALI_FORWARD,  // 校准正转: 从死区正转脱离
        DONE,          // 完成: 位置环锁位, 下一帧回 READY
    } trig_state{trig_state_e::READY};

    // --- 校准 ---
    bool     is_calibrated{false};      // 是否已完成校准
    float    trigger_offset{0};         // 编码器零点偏移 (反转到底时的角度)
    trig_state_e target_after_cali{trig_state_e::SINGLE}; // 校准后目标状态

    // --- 堵转检测 ---
    uint32_t block_start_tick{0};       // 堵转起始时刻 (0=未堵转)
    uint32_t single_start_tick{0};     // 单发开始时刻 (用于超时)

    // --- 命令缓存 (用于子状态机内部消费) ---
    bool cmd_feeder_trigger{false};
    bool cmd_fire_enable{false};
};

// =========================================================
// 4. 上下文聚合 (motor + pid + data + offsets + cmd)
// =========================================================
struct test_robot_context_t
{
    test_robot_deps_t::motor_deps_t motor;
    test_robot_deps_t::pid_deps_t   pid;
    test_robot_data_ctx_t           data;
    float                           yaw_pos_offset{0};
    float                           pitch_pos_offset{0};
    test_robot_cmd_t               *cmd{};
};

// =========================================================
// 5. ModuleParams 聚合类型
// =========================================================
struct test_robot_module_params_t
{
    using CmdType    = test_robot_cmd_t;
    using ModuleDeps = test_robot_deps_t;
    using ModuleCtx  = test_robot_context_t;
};

// =========================================================
// 6. 双轴云台 + 发射机构模块
// =========================================================
class test_robot_t final
    : public module_base_t<test_robot_t, test_robot_module_params_t>
{
    friend class module_base_t<test_robot_t, test_robot_module_params_t>;

  public:
    test_robot_t(const test_robot_t &)            = delete;
    test_robot_t &operator=(const test_robot_t &) = delete;
    using data_ctx_t           = test_robot_data_ctx_t;
    using test_robot_context_t = pyro::test_robot_context_t;

  private:
    test_robot_t();
    ~test_robot_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void     _update_feedback() override;
    void     _fsm_execute() override;

    // --- 私有控制方法 ---
    void _gimbal_control();
    void _friction_control();
    void _feeder_control();
    void _send_motor_command() const;

    // --- 拨弹盘子状态机 ---
    void _trig_fsm_execute();

    // 子状态处理
    void _trig_ready_enter();
    void _trig_ready_execute();
    void _trig_single_execute();
    void _trig_continue_execute();
    void _trig_cali_reverse_execute();
    void _trig_cali_forward_execute();
    void _trig_done_execute();

    // 切换到子状态
    void _trig_goto(decltype(test_robot_data_ctx_t::trig_state) s);

    // --- FSM 状态定义 ---
    using owner = test_robot_t;

    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *o) override;
        void execute(owner *o) override;
        void exit(owner *o) override;
    };

    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *o) override;
        void execute(owner *o) override;
        void exit(owner *o) override;
    };

    state_passive_t _state_passive;
    state_active_t  _state_active;
    fsm_t<owner>    _main_fsm;
};

} // namespace pyro

#endif // __PYRO_TEST_ROBOT_H__
