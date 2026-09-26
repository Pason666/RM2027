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
    float pitch_rate;        // Pitch轴目标角速度 rad/s (来自遥控器左摇杆Y)
    float friction_speed;    // 摩擦轮目标转速 rad/s
    float feeder_speed;      // 拨弹盘连发目标转速 rad/s (进入连发时设定)
    bool  feeder_trigger;    // 拨弹盘触发信号 (上升沿触发单发/初始化)
    bool  fire_enable;       // 连发模式使能
    bool  force_stop;        // 强制停止 (左拨杆在UP时为true)

    test_robot_cmd_t()
        : pitch_rate(0),
          friction_speed(TEST_FRICTION_DEFAULT_SPEED),
          feeder_speed(TRIGGER_CONTINUOUS_RADPS),
          feeder_trigger(false), fire_enable(false), force_stop(false)
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
        motor_base_t *pitch{nullptr};           // Pitch轴 DM4310 (CAN2)
        motor_base_t *friction[2]{nullptr};     // 摩擦轮×2 M3508 (CAN2, ID=1,2)
        motor_base_t *feeder{nullptr};          // 拨弹盘 M2006 (CAN2, ID=3)
    };

    struct pid_deps_t
    {
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
};

// =========================================================
// 3. 运行时数据上下文
// =========================================================
struct test_robot_data_ctx_t
{
    // --- 云台 ---
    float current_pitch_rad{0};
    float current_pitch_radps{0};
    float target_pitch_rad{0};
    float target_pitch_radps{0};
    float out_pitch_torque{0};
    float pitch_init_position{0};  // 上电前记录的pitch位置，用于初始化目标位置

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
        IDLE,          // 空闲: 未初始化状态
        INIT_FORWARD,  // 初始化: 正转直到卡住找零点
        READY,         // 就绪: 位置环锁位
        SINGLE,        // 单发: 位置环推进 SINGLE_SHOT_ANGLE
        CONTINUE,      // 连发: 速度环恒转速
        JAM_RECOVERY,  // 卡弹恢复: 无力一段时间后重试
        DONE,          // 完成: 位置环锁位, 下一帧回 READY
    } trig_state{trig_state_e::IDLE};

    // --- 初始化 ---
    bool     is_initialized{false};     // 是否已完成初始化
    bool     init_just_completed{false}; // 初始化刚完成标志（用于通知应用层打开摩擦轮）
    float    trigger_offset{0};         // 编码器零点偏移 (正转卡住时的角度)

    // --- 卡弹检测 ---
    uint32_t jam_start_tick{0};         // 卡弹起始时刻 (0=未卡弹)
    uint32_t jam_recovery_start_tick{0}; // 卡弹恢复开始时刻
    uint8_t  jam_recovery_count{0};     // 当前单发的卡弹恢复次数
    uint32_t single_start_tick{0};      // 单发开始时刻 (用于超时)
    trig_state_e state_before_jam{trig_state_e::READY}; // 卡弹前的状态

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

    // 供外部访问的接口
    bool is_init_just_completed() const { return _ctx.data.init_just_completed; }
    void clear_init_completed_flag() { _ctx.data.init_just_completed = false; }

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
    void _trig_idle_execute();
    void _trig_init_forward_execute();
    void _trig_ready_enter();
    void _trig_ready_execute();
    void _trig_single_execute();
    void _trig_continue_execute();
    void _trig_jam_recovery_execute();
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
