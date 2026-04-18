#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_referee.h" // 引入裁判系统头文件

#include <stdint.h>
#include <string.h>

using namespace pyro;

static pyro::hybrid_chassis_t *hybrid_chassis_ptr       = nullptr;
static pyro::hybrid_cmd_t *hybrid_cmd_ptr               = nullptr;
static pyro::hybrid_deps_t *hybrid_deps_ptr             = nullptr;

static void chassis_rxcmd();
static void chassis_dr162cmd();
static void deps_init();
static void quaternion_tx();
static void shoot_tx(); // 声明 shoot_tx 函数

__attribute__((section(".dma_heap"))) char shoot_speed[10]; // 从旧代码迁移：DMA 内存数组

extern "C"
{
    void hero_chassis_thread(void *argument)
    {
        while (true)
        {
            chassis_rxcmd();
            shoot_tx(); // 恢复：调用裁判系统和弹速发送逻辑
            // 如果后续希望由底盘板直接解算 RC，可以取消下面这行的注释
            // chassis_dr162cmd();
            quaternion_tx();
            hybrid_chassis_ptr->set_command(*hybrid_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_chassis_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(pyro::can_hub_t::which_can::can1, 0x101);
        hybrid_cmd_ptr     = new pyro::hybrid_cmd_t();
        hybrid_chassis_ptr = pyro::hybrid_chassis_t::instance();

        deps_init();
        hybrid_chassis_ptr->configure(*hybrid_deps_ptr);
        hybrid_chassis_ptr->start();

        xTaskCreate(hero_chassis_thread, "start_hero_chassis_thread", 128,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }
}

void chassis_rxcmd()
{
    std::array<uint8_t, 8> raw_data{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x101, raw_data))
    {
        // 恢复旧代码的数值：将 1.2f 恢复为 1.0f
        hybrid_cmd_ptr->vx =
            1.0f * static_cast<float>(static_cast<int8_t>(raw_data[0])) / 127.0f;
        hybrid_cmd_ptr->vy =
            1.0f * static_cast<float>(static_cast<int8_t>(raw_data[1])) / 127.0f;
        hybrid_cmd_ptr->mode =
            static_cast<pyro::cmd_base_t::mode_t>(raw_data[3] & 0x01);
        hybrid_cmd_ptr->crossing_en    = (raw_data[3] & 0x02) != 0;
        hybrid_cmd_ptr->leg_retract = (raw_data[3] & 0x04) != 0;
    }
}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        hybrid_cmd_ptr->vx          = 0;
        hybrid_cmd_ptr->vy          = 0;
        hybrid_cmd_ptr->wz          = 0;
        hybrid_cmd_ptr->delta_pitch = 0;
        hybrid_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::PASSIVE;
    }
    else
    {
        hybrid_cmd_ptr->vx          = 2.0f * vrc.axes.ly;
        hybrid_cmd_ptr->vy          = -2.0f * vrc.axes.lx;
        hybrid_cmd_ptr->delta_pitch = 0.002f * vrc.axes.ry;
        hybrid_cmd_ptr->delta_yaw   = -0.003f * vrc.axes.rx;
        hybrid_cmd_ptr->mode        = pyro::cmd_base_t::mode_t::ACTIVE;

        if (pyro::sw_pos_t::DOWN != vrc.switches.left.current_pos)
        {
            hybrid_cmd_ptr->crossing_en = true;
            if (pyro::sw_pos_t::MID == vrc.switches.left.current_pos)
            {
                hybrid_cmd_ptr->leg_retract = true;
            }
            else
            {
                hybrid_cmd_ptr->leg_retract = false;
            }
        }
        else
        {
            hybrid_cmd_ptr->crossing_en = false;
        }
    }
}

void deps_init()
{
    hybrid_deps_ptr = new pyro::hybrid_deps_t();
    hybrid_deps_ptr->motor_deps.mecanum[0] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,
                                        pyro::can_hub_t::can3); // FL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[1] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,
                                        pyro::can_hub_t::can3); // FR Wheel
    hybrid_deps_ptr->motor_deps.mecanum[2] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,
                                        pyro::can_hub_t::can3); // RL Wheel
    hybrid_deps_ptr->motor_deps.mecanum[3] =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4,
                                        pyro::can_hub_t::can3); // RR Wheel
    hybrid_deps_ptr->motor_deps.track[0] =
        new pyro::dm_motor_drv_t(0x11, 0x21, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.track[1] =
        new pyro::dm_motor_drv_t(0x12, 0x22, pyro::can_hub_t::can3);
    hybrid_deps_ptr->motor_deps.leg[0] =
        new pyro::dm_motor_drv_t(0x31, 0x41, pyro::can_hub_t::can2);
    hybrid_deps_ptr->motor_deps.leg[1] =
        new pyro::dm_motor_drv_t(0x32, 0x42, pyro::can_hub_t::can2);

    hybrid_deps_ptr->motor_deps.yaw = new pyro::dji_gm_6020_motor_drv_t(
        pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can1);

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_rotate_range(-30.0f, 30.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[0])
        ->set_torque_range(-11.0f, 11.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.track[1])
        ->set_torque_range(-11.0f, 11.0f);

    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_rotate_range(-5.655f, 5.655f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[0])
        ->set_torque_range(-27.0f, 27.0f);
    static_cast<dm_motor_drv_t *>(hybrid_deps_ptr->motor_deps.leg[1])
        ->set_torque_range(-27.0f, 27.0f);
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    hybrid_deps_ptr->pid_deps.mecanum_pid[0] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[1] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[2] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.mecanum_pid[3] =
        new pid_t(0.35f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);

    hybrid_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(5.0f, 0.0f, 0.1f, 0.0f, 10.0f, 200, 100, 4);

    hybrid_deps_ptr->pid_deps.track_pid[0] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.track_pid[1] =
        new pid_t(0.02f, 0.0001f, 0.00002f, 0.5f, 11.0f, 20, 10, 4);

    hybrid_deps_ptr->pid_deps.pitch_pid = new pid_t(
        260.0f, 0.1f, 10.0f, 20.0f, 200.0f, 200, 100, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);
    hybrid_deps_ptr->pid_deps.roll_pid = new pid_t(
        360.0f, 0.1f, 10.0f, 20.0f, 200.0f, 200, 100, 4,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);

    hybrid_deps_ptr->pid_deps.leg_pos_pid[0] =
        new pid_t(11.2f, 0.005f, 0.008f, 0.01f, 0.5f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_pos_pid[1] =
        new pid_t(11.2f, 0.005f, 0.008f, 0.01f, 0.5f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[0] =
        new pid_t(220.0f, 0.005f, 0.008f, 5.0f, 200.0f, 20, 10, 4);
    hybrid_deps_ptr->pid_deps.leg_vel_pid[1] =
        new pid_t(220.0f, 0.005f, 0.008f, 5.0f, 200.0f, 20, 10, 4);
}

/**
 * @brief 轻量级浮点数转字符函数（保留5位小数）
 * @param value 要转换的浮点数
 * @param buffer 输出的字符数组
 * @param max_len 数组最大长度（防止越界）
 */
void float_to_char_5_decimals(float value, char* buffer, int max_len)
{
    int idx = 0;

    // 1. 处理符号
    if (value < 0) {
        if (idx < max_len - 1) buffer[idx++] = '-';
        value = -value;
    }

    // 2. 分离整数和小数部分
    int int_part = (int)value;
    // 加 0.5f 用于实现最后一位的四舍五入
    int frac_part = (int)((value - (float)int_part) * 100000.0f + 0.5f);

    // 处理四舍五入导致的进位
    if (frac_part >= 100000) {
        int_part++;
        frac_part -= 100000;
    }

    // 3. 计算整数部分的位数
    int temp = int_part;
    int num_digits = 0;
    do {
        num_digits++;
        temp /= 10;
    } while (temp > 0);

    // 4. 边界安全检查：符号位 + 整数位数 + 小数点(1) + 5位小数 + 结束符(1)
    if (idx + num_digits + 1 + 5 + 1 > max_len) {
        // 如果越界（例如弹速异常到了三位数），默认安全返回全0
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    // 5. 提取整数部分（逆序写入）
    for (int i = num_digits - 1; i >= 0; i--) {
        buffer[idx + i] = '0' + (int_part % 10);
        int_part /= 10;
    }
    idx += num_digits;

    // 6. 写入小数点
    buffer[idx++] = '.';

    // 7. 提取小数部分（固定提取5位）
    for (int i = 4; i >= 0; i--) {
        buffer[idx + i] = '0' + (frac_part % 10);
        frac_part /= 10;
    }
    idx += 5;

    // 8. 添加字符串结束符
    buffer[idx] = '\0';
}

void shoot_tx()
{
    static uint16_t launching_num = 0;
    float current_speed =
        pyro::referee_drv_t::get_instance()->get_data().shoot.initial_speed;

    if (launching_num != pyro::referee_drv_t::get_instance()->get_data().shoot.launching_num)
    {
        pyro::can_tx_drv_t::clear(0x135);
        pyro::can_tx_drv_t::add_data(0x135, 32, current_speed);
        pyro::can_tx_drv_t::send(0x135, pyro::can_hub_t::get_instance()->hub_get_can_obj(pyro::can_hub_t::can1));
        launching_num = pyro::referee_drv_t::get_instance()->get_data().shoot.launching_num;

        // ================= 逻辑修改处 =================
        // 将浮点数转换为字符串并存入 DMA 堆内存中的 shoot_speed 数组
        // float_to_char_5_decimals(current_speed, shoot_speed, sizeof(shoot_speed));
        // ============================================
        // last_speed = current_speed;
    }
}

void quaternion_tx()
{
    static float q0, q1, q2, q3;
    ins_drv_t::get_instance()->get_quaternion(&q0, &q1, &q2, &q3);

    pyro::can_tx_drv_t::clear(0x103);
    const auto send_q0 = static_cast<int16_t>(q0 * 32767.0f);
    const auto send_q1 = static_cast<int16_t>(q1 * 32767.0f);
    const auto send_q2 = static_cast<int16_t>(q2 * 32767.0f);
    const auto send_q3 = static_cast<int16_t>(q3 * 32767.0f);

    can_tx_drv_t::add_data(0x103, 16, send_q0);
    can_tx_drv_t::add_data(0x103, 16, send_q1);
    can_tx_drv_t::add_data(0x103, 16, send_q2);
    can_tx_drv_t::add_data(0x103, 16, send_q3);

    can_tx_drv_t::send(
        0x103, can_hub_t::get_instance()->hub_get_can_obj(can_hub_t::can1));
}