/**
* @file pyro_board_com.cpp
 * @brief 板间通信业务逻辑层 - 底盘端 (Chassis)
 */

#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_hybrid_chassis.h"

using namespace pyro;

static TaskHandle_t board_com_task_handle = nullptr;
static board_drv_t *board_drv_ptr         = nullptr;

static void process_chassis_logic()
{
    // 1. 填充底盘状态发给云台
    auto &tx_data = board_drv_ptr->get_c2g_tx_data();
    tx_data.chassis_power = 6000; // 假数据示例
    tx_data.state_flags   = 0x01;

    // 2. 读取云台指令并应用到底盘
    if (board_drv_ptr->check_online())
    {
        const auto &rx_data = board_drv_ptr->get_g2c_rx_data();

        // 示例：此处接入底盘控制命令的映射
        // auto* hybrid_cmd_ptr = hybrid_chassis_t::instance()->get_cmd();
        // hybrid_cmd_ptr->vx = 2.0f * static_cast<float>(rx_data.vx) / 127.0f;
        // hybrid_cmd_ptr->vy = 2.0f * static_cast<float>(rx_data.vy) / 127.0f;
        // hybrid_cmd_ptr->mode = rx_data.active ? cmd_base_t::mode_t::ACTIVE : cmd_base_t::mode_t::PASSIVE;
        // hybrid_cmd_ptr->crossing_en = rx_data.track_en;
        // hybrid_cmd_ptr->leg_retract = rx_data.leg_retract;
    }
}

extern "C"
{
    void hero_board_com_thread(void *argument)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        while (true)
        {
            process_chassis_logic();
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void hero_board_com_init(void *argument)
    {
        board_drv_ptr = &board_drv_t::get_instance(board_drv_t::role_t::CHASSIS, can_hub_t::can1);
        board_drv_ptr->start_rx();

        xTaskCreate(hero_board_com_thread, "board_com_app", 256, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handle);
        vTaskDelete(nullptr);
    }
}