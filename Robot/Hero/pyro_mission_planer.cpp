#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"
extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);
    extern void hero_gimbal_init(void *argument);
    extern void hero_booster_init(void *argument);
    extern void hero_autoaim_init(void *argument);
    extern void hero_chassis_init(void *argument);


    void start_mission_planer_task(void const *argument)
    {

        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

#if BOARD == GIMBAL_BOARD
        xTaskCreate(hero_gimbal_init, "pyro_gimbal_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(hero_booster_init, "pyro_booster_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
        xTaskCreate(hero_autoaim_init, "pyro_autoaim_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#elif BOARD == CHASSIS_BOARD
        xTaskCreate(hero_chassis_init, "pyro_chassis_init", 512, nullptr,
                    configMAX_PRIORITIES - 2, nullptr);
#endif
        vTaskDelete(nullptr);

    }
}