/*
 * CyberGear电机应用 - 电流模式双环PID位置控制
 *
 * 功能：使用双环PID实现平滑的位置跟踪控制
 * - 外环：位置PID（输出目标速度）
 * - 内环：速度PID（输出目标力矩）
 * - 电机工作在电流模式
 */

#include "pyro_cybergear_motor_drv.h"
#include "pyro_algo_pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

using namespace pyro;

// ==========================================
// 全局对象
// ==========================================

static cybergear_motor_drv_t *motor = nullptr;
static pid_t *position_pid = nullptr;
static pid_t *velocity_pid = nullptr;

// ==========================================
// 控制任务
// ==========================================

static void control_task(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 初始化电流模式
    motor->init_current_mode();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 初始化PID控制器
    position_pid = new pid_t(10.0f, 0.0f, 0.5f, 5.0f, 30.0f);   // 位置环
    velocity_pid = new pid_t(0.5f, 0.01f, 0.0f, 3.0f, 12.0f);   // 速度环
    vTaskDelay(pdMS_TO_TICKS(100));

    // 控制循环参数
    const float pi = 3.14159265359f;
    float time = 0.0f;

    while (true)
    {
        // 生成目标位置：正弦波 -π/2 到 π/2
        float target_position = sinf(time) * (pi / 2.0f);

        // 读取反馈并计算控制量
        if (motor->update_feedback() == PYRO_OK)
        {
            float current_position = motor->get_current_position();
            float current_velocity = motor->get_current_rotate();

            // 外环：位置PID -> 目标速度
            float target_velocity = position_pid->calculate(target_position, current_position);

            // 内环：速度PID -> 目标力矩
            float target_torque = velocity_pid->calculate(target_velocity, current_velocity);

            // 发送控制指令
            motor->send_control(target_torque);

            // 检查故障
            auto fault = motor->get_detailed_fault();
            if (fault.motor_over_temp || fault.driver_chip_fault ||
                fault.under_voltage || fault.over_voltage)
            {
                motor->disable();
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz控制频率
        time += 0.01f;
    }

    vTaskDelete(nullptr);
}

// ==========================================
// 初始化接口
// ==========================================

extern "C" void cybergear_app_init()
{
    // 创建电机对象
    motor = new cybergear_motor_drv_t(127, 0, can_hub_t::can1);

    // 设置位置反馈范围为 [-π, π]
    const float pi = 3.14159265359f;
    motor->set_position_range(-pi, pi);

    // 创建控制任务
    xTaskCreate(control_task, "cybergear_ctrl", 1024, nullptr,
                configMAX_PRIORITIES - 3, nullptr);
}
