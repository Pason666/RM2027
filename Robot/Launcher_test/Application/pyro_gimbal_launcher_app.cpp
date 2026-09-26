/**
 * @file pyro_gimbal_launcher_app.cpp
 * @brief Test Robot 云台 + 发射机构应用层
 *
 * 遥控器操作:
 *   SW_R UP    → PASSIVE (全部电机失能)
 *   SW_R MID   → ACTIVE  (使能整个系统, 云台跟随)
 *   SW_R DOWN  → ACTIVE  (使能整个系统, 云台跟随)
 *
 *   左摇杆Y → Pitch 角速度控制
 *
 *   SW_L UP→MID   → 拨弹盘正转卡住初始化零点, 然后摩擦轮打开
 *   SW_L MID→UP   → 摩擦轮关闭
 *   SW_L DOWN→MID 或 DOWN→UP → 触发单发射击
 *   SW_L DOWN 维持 ≥ 800ms → 进入连发模式
 *
 * 硬件映射:
 *   CAN2: Pitch DM4310 + Friction L M3508 (ID=1) + Friction R M3508 (ID=2) + Feeder M2006 (ID=3)
 */

#include "pyro_module_base.h"
#include "pyro_test_robot.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"

using namespace pyro;

// ========== 任务通知事件位 ==========
constexpr uint32_t EVENT_BIT_INIT     = (1u << 0); // SW_L UP→MID: 初始化拨弹盘
constexpr uint32_t EVENT_BIT_FRIC_OFF = (1u << 1); // SW_L MID→UP: 关闭摩擦轮
constexpr uint32_t EVENT_BIT_FIRE     = (1u << 2); // SW_L DOWN→MID/UP: 单发触发

// ========== 全局句柄 ==========
static TaskHandle_t      test_robot_task_handle = nullptr;
static test_robot_t     *test_robot_ptr         = nullptr;
static test_robot_cmd_t *test_robot_cmd_ptr     = nullptr;
static test_robot_deps_t *test_robot_deps_ptr   = nullptr;

// ========== 函数声明 ==========
static void deps_init();

// ========== 遥控器数据滤波 ==========
static float rcdata_filter(float data)
{
    if (fabsf(data) < 0.1f)
        return 0.0f;
    return data;
}

/// 摇杆满偏时云台角速度 (rad/s): Pitch ≈ 180°/s
constexpr float GIMBAL_PITCH_RATE_SCALE = PI * 0.65f;

/// 左拨杆维持在 DOWN 超过此值进入连发模式 (ms)
constexpr uint32_t BURST_FIRE_HOLD_MS = 800;

// ========== 依赖初始化 (电机 + PID) ==========
static void deps_init()
{
    test_robot_deps_ptr = new test_robot_deps_t();

    // ==================== CAN2: 云台电机 ====================
    // Pitch轴: DM4310, CAN2 (MIT 协议)
    test_robot_deps_ptr->motor_deps.pitch =
        new dm_motor_drv_t(0x01, 0x00, can_hub_t::can2);
    static_cast<dm_motor_drv_t *>(test_robot_deps_ptr->motor_deps.pitch)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(test_robot_deps_ptr->motor_deps.pitch)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(test_robot_deps_ptr->motor_deps.pitch)
        ->set_torque_range(-7.0f, 7.0f);

    // ==================== CAN2: 发射机构电机 ====================
    // 摩擦轮L: M3508, CAN2, ID=1
    test_robot_deps_ptr->motor_deps.friction[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                   can_hub_t::can2);

    // 摩擦轮R: M3508, CAN2, ID=2
    test_robot_deps_ptr->motor_deps.friction[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                   can_hub_t::can2);

    // 拨弹盘: M2006, CAN2, ID=3
    test_robot_deps_ptr->motor_deps.feeder =
        new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                   can_hub_t::can2);

    // ==================== PID 分配 ====================

    // --- Pitch轴: 位置环 + 速度环 (DM4310) ---
    test_robot_deps_ptr->pid_deps.pitch_pos_pid =
        new pid_t(30.0f, 0.08f, 0.2f, 0.8f, 12);
    test_robot_deps_ptr->pid_deps.pitch_spd_pid =
        new pid_t(0.6f, 0.0f, 0.001f, 0.5f, 7.0f);

    // --- 摩擦轮: 速度环 ×2 (M3508) ---
    for (int i = 0; i < 2; i++)
        test_robot_deps_ptr->pid_deps.friction_spd_pid[i] =
            new pid_t(0.5f, 0.0f, 0.0f, 0.8f, 20.0f);

    // --- 拨弹盘: 位置环 + 速度环 (M2006) ---
    // 单发需要较大力矩推动弹丸，提高P值和输出限幅
    test_robot_deps_ptr->pid_deps.feeder_pos_pid =
        new pid_t(40.0f, 1.0f, 0.0f, 5.0f, 25.0f);
    test_robot_deps_ptr->pid_deps.feeder_spd_pid =
        new pid_t(2.5f, 0.0f, 0.0f, 8.0f, 10.0f);
}

// =========================================================
// 外部C接口 (FreeRTOS任务入口)
// =========================================================
extern "C"
{

void test_robot_thread(void *argument)
{
    static uint32_t sw_l_down_ticks = 0; // DOWN 状态累计 ms

    while (true)
    {
        // 1ms 超时轮询: 事件唤醒或超时后继续处理摇杆
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, pdMS_TO_TICKS(1));

        // ---- 检查遥控器是否在线 ----
        if (!dr16_drv_t::instance().check_online())
        {
            // 遥控器离线: 强制进入 PASSIVE 模式, 关闭所有输出
            test_robot_cmd_ptr->mode           = cmd_base_t::mode_t::PASSIVE;
            test_robot_cmd_ptr->pitch_rate     = 0.0f;
            test_robot_cmd_ptr->friction_speed = 0.0f;
            test_robot_cmd_ptr->feeder_trigger = false;
            test_robot_cmd_ptr->fire_enable    = false;
            test_robot_cmd_ptr->feeder_speed   = 0.0f;
            sw_l_down_ticks = 0;

            test_robot_ptr->set_command(*test_robot_cmd_ptr);
            continue;
        }

        // ---- 处理 RC 数据 (加读锁) ----
        read_scope_lock lock(rc_drv_t::get_lock());
        const auto &rc = rc_drv_t::read();

        sw_pos_t sw_r = rc.switches.right.current_pos;
        sw_pos_t sw_l = rc.switches.left.current_pos;

        // ---- 处理左拨杆边沿事件 ----
        // 默认情况下trigger为false，只在事件发生时置true
        test_robot_cmd_ptr->feeder_trigger = false;

        if (notify_val & EVENT_BIT_INIT)
        {
            // UP→MID: 触发初始化, 初始化完成后摩擦轮会自动打开
            test_robot_cmd_ptr->feeder_trigger = true;
        }

        if (notify_val & EVENT_BIT_FIRE)
        {
            // DOWN→MID/UP: 触发单发
            test_robot_cmd_ptr->feeder_trigger = true;
        }

        // ---- 检查初始化是否刚完成，如果完成则打开摩擦轮 ----
        if (test_robot_ptr->is_init_just_completed())
        {
            test_robot_cmd_ptr->friction_speed = TEST_FRICTION_DEFAULT_SPEED;
            test_robot_ptr->clear_init_completed_flag();  // 清除标志，避免重复处理
        }

        if (notify_val & EVENT_BIT_FRIC_OFF)
            test_robot_cmd_ptr->friction_speed = 0.0f;

        // ---- 左拨杆UP: 强制停止拨弹盘 ----
        test_robot_cmd_ptr->force_stop = (sw_pos_t::UP == sw_l);

        // ---- DOWN 维持计时 → 连发 ----
        if (sw_pos_t::DOWN == sw_l)
        {
            sw_l_down_ticks++;
            if (sw_l_down_ticks >= BURST_FIRE_HOLD_MS)
            {
                test_robot_cmd_ptr->fire_enable  = true;
                test_robot_cmd_ptr->feeder_speed = TRIGGER_CONTINUOUS_RADPS;
            }
        }
        else
        {
            sw_l_down_ticks = 0;
            // 离开DOWN位置，清除连发
            test_robot_cmd_ptr->fire_enable  = false;
            test_robot_cmd_ptr->feeder_speed = 0.0f;
        }

        // ---- 右开关: 模式切换 ----
        if (sw_pos_t::UP == sw_r)
        {
            test_robot_cmd_ptr->mode           = cmd_base_t::mode_t::PASSIVE;
            test_robot_cmd_ptr->pitch_rate     = 0.0f;
            test_robot_cmd_ptr->friction_speed = 0.0f;
            test_robot_cmd_ptr->feeder_trigger = false;
            test_robot_cmd_ptr->fire_enable    = false;
        }
        else // SW_R MID 或 DOWN → 使能系统
        {
            test_robot_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;

            // 左摇杆Y → Pitch 角速度
            test_robot_cmd_ptr->pitch_rate =
                -rcdata_filter(rc.axes.ly) * GIMBAL_PITCH_RATE_SCALE;
        }

        test_robot_ptr->set_command(*test_robot_cmd_ptr);
    }
}

void test_robot_init(void *argument)
{
    test_robot_cmd_ptr = new test_robot_cmd_t();
    test_robot_ptr     = test_robot_t::instance();

    deps_init();
    test_robot_ptr->configure(*test_robot_deps_ptr);
    test_robot_ptr->start();

    // 创建遥控器命令处理线程
    xTaskCreate(test_robot_thread, "test_robot_thread", 512,
                nullptr, configMAX_PRIORITIES - 1,
                &test_robot_task_handle);

    // ---- 订阅左拨杆边沿事件 ----
    auto &vrc = rc_drv_t::read();

    sw_broker::subscribe(&vrc.switches.left, sw_event_t::UP_TO_MID,
                         test_robot_task_handle, EVENT_BIT_INIT);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_UP,
                         test_robot_task_handle, EVENT_BIT_FRIC_OFF);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::DOWN_TO_MID,
                         test_robot_task_handle, EVENT_BIT_FIRE);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::DOWN_TO_UP,
                         test_robot_task_handle, EVENT_BIT_FIRE);

    vTaskDelete(nullptr);
}

} // extern "C"
