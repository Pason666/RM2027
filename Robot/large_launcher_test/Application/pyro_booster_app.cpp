#include "pyro_module_base.h"
#include "pyro_large_launcher.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_referee.h"
#include "pyro_algo_pid.h"

using namespace pyro;

// ========== 任务通知事件位 ==========
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE      = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE             = (1 << 1);
constexpr uint32_t EVENT_BIT_TRIGGER_RESET    = (1 << 2);
constexpr uint32_t EVENT_BIT_SHOOT_DATA_RESET = (1 << 3);

// ========== 全局句柄 ==========
static TaskHandle_t booster_task_handle   = nullptr;
static tri_booster_t *tri_booster_ptr         = nullptr;
static tri_booster_cmd_t *tri_booster_cmd_ptr = nullptr;
static tri_deps_t *tri_deps_ptr               = nullptr;

// ========== 函数声明 ==========
static void booster_dr16_cmd(uint32_t notify_val);
static void booster_vt03_cmd(uint32_t notify_val);

// ========== 函数声明 ==========
static void deps_init();

// ========== 依赖初始化 (电机 + PID) ==========
static void deps_init()
{
    tri_deps_ptr = new tri_deps_t();

    // ==================== CAN2: 三个摩擦轮 M3508 ====================
    tri_deps_ptr->motor_deps.fric_wheels[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can2);
    tri_deps_ptr->motor_deps.fric_wheels[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can2);
    tri_deps_ptr->motor_deps.fric_wheels[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can2);

    // ==================== CAN1: 拨弹盘 DM4310 ====================
    tri_deps_ptr->motor_deps.trigger_wheel =
        new dm_motor_drv_t(0x51, 0x61, can_hub_t::can1);

    static_cast<dm_motor_drv_t *>(tri_deps_ptr->motor_deps.trigger_wheel)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(tri_deps_ptr->motor_deps.trigger_wheel)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(tri_deps_ptr->motor_deps.trigger_wheel)
        ->set_torque_range(-7.0f, 7.0f);

    // ==================== PID 分配 ====================
    // --- 摩擦轮: 速度环 ×3 (M3508) ---
    tri_deps_ptr->pid_deps.fric_pid[0] =
        new pyro::pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 1, 80, 1, 4);
    tri_deps_ptr->pid_deps.fric_pid[1] =
        new pyro::pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 1, 80, 1, 4);
    tri_deps_ptr->pid_deps.fric_pid[2] =
        new pyro::pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 1, 80, 1, 4);

    // --- 拨弹盘: 位置环 + 速度环 (DM4310) ---
    tri_deps_ptr->pid_deps.trigger_pos_pid =
        new pyro::pid_t(8.0f, 0.03f, 0.0015f, 0.3f, 2.0f, 40, 1, 20, 1, 4);
    tri_deps_ptr->pid_deps.trigger_spd_pid =
        new pyro::pid_t(0.9f, 0.9f, 0.0015f, 1.0f, 5.0f, 30, 1, 20, 1, 4);
}

// =========================================================
// DR16 遥控器命令处理
// =========================================================
void booster_dr16_cmd(uint32_t notify_val)
{
    read_scope_lock lock(rc_drv_t::get_lock());
    auto &vrc = rc_drv_t::read();

    if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        tri_booster_cmd_ptr->mode    = cmd_base_t::mode_t::PASSIVE;
        tri_booster_cmd_ptr->fric_on = false;
        return;
    }

    tri_booster_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;

    if (notify_val & EVENT_BIT_FRIC_TOGGLE)
    {
        tri_booster_cmd_ptr->fric_on = !tri_booster_cmd_ptr->fric_on;
    }

    if (sw_pos_t::MID == vrc.switches.right.current_pos)
    {
        if (notify_val & EVENT_BIT_FIRE)
        {
            tri_booster_cmd_ptr->fire_count++;
        }
    }
}

// =========================================================
// VT03 遥控器命令处理（仅保留摇杆和开关，移除键鼠）
// =========================================================
void booster_vt03_cmd(uint32_t notify_val)
{
    read_scope_lock lock(rc_drv_t::get_lock());
    auto &vrc = rc_drv_t::read();

    if (sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        tri_booster_cmd_ptr->mode    = cmd_base_t::mode_t::PASSIVE;
        tri_booster_cmd_ptr->fric_on = false;
        return;
    }

    tri_booster_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;

    // 摩擦轮开关切换（通过 Fn_L 按钮）
    if (notify_val & EVENT_BIT_FRIC_TOGGLE)
    {
        tri_booster_cmd_ptr->fric_on = !tri_booster_cmd_ptr->fric_on;
    }

    // 单发（通过扳机）
    if (notify_val & EVENT_BIT_FIRE)
    {
        tri_booster_cmd_ptr->fire_count++;
    }

    // 拨弹盘归位（通过左摇杆按下）
    if (notify_val & EVENT_BIT_TRIGGER_RESET)
    {
        tri_booster_cmd_ptr->reset_count++;
    }

    // 重置弹速数据（通过右摇杆按下）
    if (notify_val & EVENT_BIT_SHOOT_DATA_RESET)
    {
        tri_booster_cmd_ptr->shoot_data_reset_count++;
    }
}

// =========================================================
// 外部C接口 (FreeRTOS任务入口)
// =========================================================
extern "C"
{

void booster_thread(void *argument)
{
    while (true)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

        // ---- 检查遥控器是否在线 ----
        if (vt03_drv_t::instance().check_online())
        {
            booster_vt03_cmd(notify_val);
        }
        else if (dr16_drv_t::instance().check_online())
        {
            booster_dr16_cmd(notify_val);
        }
        else
        {
            // 遥控器离线
            tri_booster_cmd_ptr->mode    = cmd_base_t::mode_t::PASSIVE;
            tri_booster_cmd_ptr->fric_on = false;
        }

        tri_booster_ptr->set_command(*tri_booster_cmd_ptr);
        vTaskDelay(1);
    }
}

void large_launcher_init(void *argument)
{
    tri_booster_cmd_ptr = new tri_booster_cmd_t();
    tri_booster_ptr     = tri_booster_t::instance();

    deps_init();
    tri_booster_ptr->configure(*tri_deps_ptr);

    xTaskCreate(booster_thread, "booster_thread", 512,
                nullptr, configMAX_PRIORITIES - 1,
                &booster_task_handle);

    auto &vrc = rc_drv_t::read();

    // ---- VT03 遥控器按钮订阅（仅摇杆按钮，无键鼠） ----
    btn_broker::subscribe(&vrc.buttons.fn_l, btn_event_t::PRESS_DOWN,
                          booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
    btn_broker::subscribe(&vrc.buttons.trigger, btn_event_t::PRESS_DOWN,
                          booster_task_handle, EVENT_BIT_FIRE);
    btn_broker::subscribe(&vrc.buttons.press_l, btn_event_t::PRESS_DOWN,
                          booster_task_handle, EVENT_BIT_TRIGGER_RESET);
    btn_broker::subscribe(&vrc.buttons.press_r, btn_event_t::PRESS_DOWN,
                          booster_task_handle, EVENT_BIT_SHOOT_DATA_RESET);

    // ---- DR16 遥控器拨杆订阅 ----
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::UP_TO_MID,
                         booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_DOWN,
                         booster_task_handle, EVENT_BIT_FIRE);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::MID_TO_UP,
                         booster_task_handle, EVENT_BIT_TRIGGER_RESET);
    sw_broker::subscribe(&vrc.switches.left, sw_event_t::DOWN_TO_MID,
                         booster_task_handle, EVENT_BIT_SHOOT_DATA_RESET);

    tri_booster_ptr->start();
    vTaskDelete(nullptr);
}

} // extern "C"
