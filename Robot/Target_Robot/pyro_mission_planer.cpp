#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void target_chassis_init(void *argument);

    void start_mission_planer_task(void const *argument)
    {
        // 1. 硬件初始化 (CAN, DR16, INS, DWT)
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

        // 等待硬件初始化完成
        vTaskDelay(10);

        // 2. 底盘模块初始化 (电机配置 + PID配置 + 启动控制任务)
        xTaskCreate(target_chassis_init, "target_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);

        vTaskDelete(nullptr);
    }
}
