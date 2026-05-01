/**
* @file pyro_board_com.cpp
 * @brief 板间通信业务逻辑层 - 云台端 (Gimbal)
 */

#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"

using namespace pyro;

static TaskHandle_t board_com_task_handle = nullptr;
static board_drv_t *board_drv_ptr         = nullptr;

static void process_gimbal_logic()
{
    auto &tx_data = board_drv_ptr->get_g2c_tx_data();

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        tx_data.active      = false;
        tx_data.vx          = 0;
        tx_data.vy          = 0;
        tx_data.wz          = 0;
        tx_data.track_en    = false;
        tx_data.leg_retract = false;
    }
    else
    {
        tx_data.active      = true;
        tx_data.vx          = static_cast<int8_t>(vrc.axes.ly * 127);
        tx_data.vy          = static_cast<int8_t>(-vrc.axes.lx * 127);
        tx_data.wz          = 0;
        tx_data.track_en    = false;
        tx_data.leg_retract = false;
    }

    if (board_drv_ptr->check_online())
    {
        // 预留：读取底盘发来的反馈信息并处理
        // const auto &rx_data = board_drv_ptr->get_c2g_rx_data();
    }
}

extern "C"
{
    void hero_board_com_thread(void *argument)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        while (true)
        {
            process_gimbal_logic();
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void hero_board_com_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::GIMBAL, can_hub_t::can1);
        board_drv_ptr->start_rx();

        xTaskCreate(hero_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handle);
        vTaskDelete(nullptr);
    }
}