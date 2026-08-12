#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void test_robot_init(void *argument);

    void start_mission_planer_task(void const *argument)
    {
        // 1. 硬件初始化 (CAN1, CAN2, DR16, INS, DWT)
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

        // 等待硬件初始化完成
        vTaskDelay(10);

        // 2. 云台+发射机构模块初始化
        xTaskCreate(test_robot_init, "test_robot_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);

        vTaskDelete(nullptr);
    }
}
