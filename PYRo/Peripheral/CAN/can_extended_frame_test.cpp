/*
 * CAN 扩展帧测试程序
 * 用于验证标准帧和扩展帧的收发功能
 */

#include "pyro_can_drv.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>

using namespace pyro;

// 测试消息缓冲区
static can_msg_buffer_t *std_msg_buffer = nullptr;   // 标准帧 0x123
static can_msg_buffer_t *ext_msg_buffer = nullptr;   // 扩展帧 0x18FF1234

// 测试任务句柄
static TaskHandle_t test_task_handle = nullptr;

extern "C"
{
    extern can_drv_t *can1_drv;

    // CAN 扩展帧测试任务
    void can_extended_test_task(void *argument)
    {
        // 初始化消息缓冲区
        std_msg_buffer = new can_msg_buffer_t(0x123, can_msg_buffer_t::STANDARD_ID);
        ext_msg_buffer = new can_msg_buffer_t(0x18FF1234, can_msg_buffer_t::EXTENDED_ID);

        // 注册接收消息
        can1_drv->register_rx_msg(std_msg_buffer);
        can1_drv->register_rx_msg(ext_msg_buffer);

        uint32_t counter = 0;
        std::array<uint8_t, 8> rx_data{};

        while (true)
        {
            // 每 500ms 发送一次测试数据
            vTaskDelay(500);

            // 准备测试数据
            uint8_t tx_data[8];
            tx_data[0] = (counter >> 24) & 0xFF;
            tx_data[1] = (counter >> 16) & 0xFF;
            tx_data[2] = (counter >> 8) & 0xFF;
            tx_data[3] = counter & 0xFF;
            tx_data[4] = 0xAA;
            tx_data[5] = 0xBB;
            tx_data[6] = 0xCC;
            tx_data[7] = 0xDD;

            // 发送标准帧 (ID: 0x456)
            can1_drv->send_msg(0x456, tx_data, can_msg_buffer_t::STANDARD_ID);

            // 发送扩展帧 (ID: 0x18FF5678)
            can1_drv->send_msg(0x18FF5678, tx_data, can_msg_buffer_t::EXTENDED_ID);

            // 检查是否接收到标准帧
            if (std_msg_buffer->is_fresh())
            {
                if (std_msg_buffer->get_data(rx_data))
                {
                    // 这里可以添加调试输出或者通过 UART 发送
                    // printf("STD RX [0x123]: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    //        rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                    //        rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
                }
                std_msg_buffer->mark_read();
            }

            // 检查是否接收到扩展帧
            if (ext_msg_buffer->is_fresh())
            {
                if (ext_msg_buffer->get_data(rx_data))
                {
                    // printf("EXT RX [0x18FF1234]: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    //        rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                    //        rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
                }
                ext_msg_buffer->mark_read();
            }

            counter++;
        }
    }

    // 启动测试任务
    void start_can_extended_test()
    {
        xTaskCreate(can_extended_test_task, "can_ext_test", 512, nullptr,
                    configMAX_PRIORITIES - 3, &test_task_handle);
    }
}
