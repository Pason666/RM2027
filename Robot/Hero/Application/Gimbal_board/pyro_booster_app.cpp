#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_quad_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_autoaim_drv.h" // 新增：引入自瞄驱动头文件

using namespace pyro;

extern float read_time;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE              = (1 << 0);
constexpr uint32_t EVENT_BIT_FIRE                     = (1 << 1);

static TaskHandle_t booster_task_handle               = nullptr;
static pyro::quad_booster_t *quad_booster_ptr         = nullptr;
static pyro::quad_booster_cmd_t *quad_booster_cmd_ptr = nullptr;
static pyro::quad_deps_t *quad_deps_ptr               = nullptr;

static void deps_init();

extern "C"
{
    void booster_dr162cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
            // 移除手动清零，交由底层状态机自动同步处理
            return;
        }

        quad_booster_cmd_ptr->mode         = pyro::cmd_base_t::mode_t::ACTIVE;
        quad_booster_cmd_ptr->target_speed = 11.5f; // 可调节

        // 根据通知处理摩擦轮翻转
        if (notify_val & EVENT_BIT_FRIC_TOGGLE)
        {
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
        }

        // 仅在手动模式（中档）下响应射击
        if (pyro::sw_pos_t::MID == vrc.switches.right.current_pos)
        {
            if (notify_val & EVENT_BIT_FIRE)
            {
                // 发射脉冲触发时，计数器自增
                quad_booster_cmd_ptr->fire_count++;
            }
        }
    }

    void booster_vt032cmd(uint32_t notify_val)
    {
        pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        static uint8_t last_pc_fire = 0; // 用于 PC 开火命令的边沿检测

        if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
        {
            quad_booster_cmd_ptr->mode    = pyro::cmd_base_t::mode_t::PASSIVE;
            quad_booster_cmd_ptr->fric_on = false;
            last_pc_fire                  = 0;
            // 移除手动清零，交由底层状态机自动同步处理
            return;
        }

        quad_booster_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;

        if (notify_val & EVENT_BIT_FRIC_TOGGLE)
        {
            quad_booster_cmd_ptr->fric_on = !quad_booster_cmd_ptr->fric_on;
        }

        // --- 新增：判断拨动到 DOWN (右侧) 时进入自瞄状态并解析自瞄数据 ---
        if (pyro::sw_pos_t::DOWN == vrc.switches.gear.current_pos)
        {
            if (pyro::autoaim_drv_t::get_instance().check_online())
            {
                const auto &rx_data = pyro::autoaim_drv_t::get_instance().get_target_data();

                // 覆盖弹速：若视觉下发了有效弹速则应用，否则兜底
                // if (rx_data.avg_speed > 7.5f) {
                //     quad_booster_cmd_ptr->target_speed = 11.7f;
                // } else {
                    quad_booster_cmd_ptr->target_speed = 16.2f;
                // }

                // 视觉开火信号边沿检测转换为内部拨弹计数器增量
                if (rx_data.fire && !last_pc_fire)
                {
                    quad_booster_cmd_ptr->fire_count++;
                }
                last_pc_fire = rx_data.fire;
            }
            else // 视觉离线兜底
            {
                quad_booster_cmd_ptr->target_speed = 16.2f;
                last_pc_fire = 0;
            }
        }
        else // MID 档位为纯手动控制
        {
            quad_booster_cmd_ptr->target_speed = 16.2f; // 可调节
            last_pc_fire = 0;
        }

        quad_booster_cmd_ptr->trig_target_spd = 14.0f * vrc.axes.rx;

        // 处理手柄与键鼠单发脉冲 (允许手控补刀)
        if (notify_val & EVENT_BIT_FIRE)
        {
            quad_booster_cmd_ptr->fire_count++;
        }
    }

    void hero_booster_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            // 我们现在依赖计数器，无需每帧强制复位

            if (vt03_drv_t::instance().check_online())
            {
                booster_vt032cmd(notify_val);
            }
            else if (dr16_drv_t::instance().check_online())
            {
                booster_dr162cmd(notify_val);
            }
            quad_booster_ptr->set_command(*quad_booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_booster_init(void *argument)
    {
        quad_booster_ptr     = pyro::quad_booster_t::instance();
        quad_booster_cmd_ptr = new pyro::quad_booster_cmd_t();

        deps_init();
        quad_booster_ptr->configure(*quad_deps_ptr);

        xTaskCreate(hero_booster_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &booster_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        // --- VT03 按键绑定 ---
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.keys.q, pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.press_l,
                                    pyro::btn_event_t::PRESS_DOWN,
                                    booster_task_handle, EVENT_BIT_FIRE);

        // --- DR16 拨杆绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::UP_TO_MID,
                                   booster_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::sw_broker::subscribe(&vrc.switches.left,
                                   pyro::sw_event_t::DOWN_TO_MID,
                                   booster_task_handle, EVENT_BIT_FIRE);

        quad_booster_ptr->start();
        vTaskDelete(nullptr);
    }
}

void deps_init()
{
    quad_deps_ptr = new pyro::quad_deps_t();
    quad_deps_ptr->motor_deps.fric_wheels[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can2); // Fric 1
    quad_deps_ptr->motor_deps.fric_wheels[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can2); // Fric 2
    quad_deps_ptr->motor_deps.fric_wheels[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can2); // Fric 3
    quad_deps_ptr->motor_deps.fric_wheels[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can2); // Fric 4
    quad_deps_ptr->motor_deps.trigger_wheel =
        new pyro::dm_motor_drv_t(0x51, 0x61, pyro::can_hub_t::can1);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(quad_deps_ptr->motor_deps.trigger_wheel)
        ->set_torque_range(-7.0f, 7.0f);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    quad_deps_ptr->pid_deps.fric_pid[0] =
        new pid_t(6.415f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    // quad_deps_ptr->pid_deps.fric_pid[0] =
    //     new pid_t(6.402f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[1] =
        new pid_t(11.322f, 0.03f, 0.004f, 2.5f, 20, 240, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[2] =
        new pid_t(6.48f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    // quad_deps_ptr->pid_deps.fric_pid[2] =
    //     new pid_t(6.442f, 0.02f, 0.02f, 2.5f, 20, 320, 80, 4);
    quad_deps_ptr->pid_deps.fric_pid[3] =
        new pid_t(11.315f, 0.03f, 0.004f, 2.5f, 20, 240, 80, 4);

    quad_deps_ptr->pid_deps.trigger_pos_pid =
        new pid_t(14.0f, 0.03f, 0.0015f, 0.5f, 14.0f, 40, 20, 4);
    quad_deps_ptr->pid_deps.trigger_spd_pid =
        new pid_t(0.6f, 0.02f, 0.0015f, 0.7f, 7.0f, 30, 20, 4);
}

