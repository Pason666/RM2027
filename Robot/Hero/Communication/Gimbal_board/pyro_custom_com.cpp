/**
 * @file pyro_custom_com.cpp
 * @brief Custom Client Forwarding Task
 * 负责处理 PC 与 裁判系统图传链路（Image Link）之间的透明双向转发：
 * 1. PC (rx_data) -> 裁判系统 (0x0310) （触发式：仅收到新数据时发送）
 * 2. 裁判系统 (0x0311) -> PC (tx_data)
 */

#include "pyro_custom_drv.h"
#include "pyro_image_drv.h" // 你的图传链路驱动

#include <cstring>

using namespace pyro;

/* ========================================================================== */
/* Static Variables                                                           */
/* ========================================================================== */
static TaskHandle_t custom_app_task_handle = nullptr;
static custom_drv_t *custom_drv_ptr = nullptr;
static image_drv_t *image_drv_ptr = nullptr;

/* ========================================================================== */
/* Helper Function Declarations                                               */
/* ========================================================================== */
static void forward_pc_to_referee();
static void forward_referee_to_pc();

/* ========================================================================== */
/* FreeRTOS Task Entries                                                      */
/* ========================================================================== */
extern "C"
{
    void hero_custom_app_thread(void *argument)
    {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true)
        {
            // 1. 将 PC 上位机发来的自定义指令转发至裁判系统图传链路 (仅有新数据时触发)
            forward_pc_to_referee();

            // 2. 将裁判系统图传链路收到的自定义指令转发至 PC 上位机
            forward_referee_to_pc();

            // 3. 延时让出 CPU
            // 裁判系统规则要求 0x0310 最大 50Hz (20ms)，0x0311 最大 75Hz (13ms)
            // 此处采用 20ms 轮询检测，防止发包频率超出图传带宽限制被服务器拉黑
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void hero_custom_init(void *argument)
    {
        custom_drv_ptr = &pyro::custom_drv_t::get_instance();
        image_drv_ptr = &pyro::image_drv_t::get_instance();

        // 启动 PC 串口的接收和解析任务
        custom_drv_ptr->start_rx();

        // 创建应用层数据透传线程
        xTaskCreate(hero_custom_app_thread, "custom_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 4, &custom_app_task_handle);

        // 初始化完成，删除自身
        vTaskDelete(nullptr);
    }
}

/* ========================================================================== */
/* Helper Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief 将 PC 数据 (包含 seq, timestamp 和 290 字节 data，共 300 字节) 透传给裁判系统 (0x0310)
 */
static void forward_pc_to_referee()
{
    // 如果 PC 不在线，或者 没有新数据，则不向裁判系统发送以节省链路带宽
    if (!custom_drv_ptr->check_online() || !custom_drv_ptr->has_new_data())
        return;

    // 已检测到新数据，将其清除以备下次使用
    custom_drv_ptr->clear_new_data_flag();

    // 从 PC 驱动获取最新接收的数据 (300 字节)
    const auto &pc_rx_data = custom_drv_ptr->get_rx_data();

    // 拿到图传链路的 DMA 零拷贝发送句柄 (也为 300 字节)
    auto &image_tx_data = image_drv_ptr->get_client_tx_data();

    // 整体拷贝包含 seq, timestamp, data 在内的整整 300 字节
    memcpy(image_tx_data.data, &pc_rx_data, sizeof(pyro::custom_drv_t::rx_data_t));

    // 触发图传链路打包并发送 0x0310 数据
    image_drv_ptr->send_client_data();
}

/**
 * @brief 将裁判系统数据 (最大 30 字节, 0x0311) 透传给 PC
 */
static void forward_referee_to_pc()
{
    // 检查图传链路是否在线
    if (!image_drv_ptr->is_online()) return;

    // 注意：如果需要，图传链路也可以增加类似 `has_new_data` 的机制，
    // 这里假定原有逻辑（若图传有更新，或一直发送最新状态）

    // 从图传驱动获取 0x0311 下发指令的只读指针
    const uint8_t* referee_rx_data = image_drv_ptr->get_client_rx_cmd_data();

    // 拿到发往 PC 驱动的待发送数据载荷引用
    auto &pc_tx_data = custom_drv_ptr->get_tx_data();

    // 拷贝裁判系统数据到 PC 发送区
    memcpy(pc_tx_data.data, referee_rx_data, sizeof(pc_tx_data.data));

    // 触发 PC 链路打包并发送
    custom_drv_ptr->send_data();
}