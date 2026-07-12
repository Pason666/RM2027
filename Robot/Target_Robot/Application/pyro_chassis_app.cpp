/**
 * @file pyro_chassis_app.cpp
 * @brief Target Robot 底盘应用层
 *
 * 功能:
 *   - DR16 遥控器数据读取与解析
 *   - 电机/PID 依赖注入配置 (deps_init)
 *   - 遥控器模式切换:
 *       SW_R UP   → PASSIVE (全部电机失能)
 *       SW_R MID  → ACTIVE, 跟随 (rx→target_imu增量→yaw电机IMU PID+follow_yaw_pid对齐)
 *       SW_R DOWN → ACTIVE, 小陀螺 (底盘固定自旋wz=2, rx→云台朝向增量)
 *       SW_L MID  → 强制自旋正转 (wz=2.0)
 *       SW_L DOWN → 强制自旋反转 (wz=-2.0)
 *   - 创建底盘控制线程
 *
 * 对应原工程: PYRo-uCtrl-Unity/PYRo/Application/Mission/Target/pyro_target_chassis_app.cpp
 */

#include "pyro_module_base.h"
#include "pyro_target_robot.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_ins.h"

#include <cstdint>

using namespace pyro;

// ========== 全局句柄 ==========
static target_robot_t *target_robot_ptr = nullptr;
static target_cmd_t *target_cmd_ptr   = nullptr;
static target_deps_t *target_deps_ptr = nullptr;
static ins_drv_t *ins_drv             = nullptr;

// ========== 函数声明 ==========
static void chassis_rc2cmd();
static void deps_init();

// ========== 遥控器数据滤波 ==========
static float rcdata_filter(float data)
{
    if (fabsf(data) < 0.1f)
        return 0.0f;
    return data;
}

/// 摇杆满偏时 IMU 目标角度变化率 (rad/s)，即 360°/s
constexpr float IMU_RATE_SCALE = PI * 1.3f;
/// 控制循环周期 (s)，与 vTaskDelay(1) 对应
constexpr float CTRL_DT = 0.001f;

// ========== 遥控器命令解析 ==========
static void chassis_rc2cmd()
{
    // 读取 DR16 遥控器数据
    const auto &rc = rc_drv_t::read();

    // 获取开关状态
    sw_pos_t sw_r = rc.switches.right.current_pos;
    sw_pos_t sw_l = rc.switches.left.current_pos;

    // --- 右开关: 模式切换 ---
    if (sw_pos_t::UP == sw_r)
    {
        // PASSIVE: 全部失能
        target_cmd_ptr->mode       = cmd_base_t::mode_t::PASSIVE;
        target_cmd_ptr->follow_yaw = false;
        target_cmd_ptr->timestamp  = 0;
        target_cmd_ptr->vx         = 0.0f;
        target_cmd_ptr->vy         = 0.0f;
        target_cmd_ptr->wz         = 0.0f;
        target_cmd_ptr->wz2        = 0.0f;
    }
    else if (sw_pos_t::MID == sw_r)
    {
        // ACTIVE: 跟随模式 — 摇杆rx做IMU位置增量, 底盘follow_yaw_pid自动对齐
        target_cmd_ptr->mode             = cmd_base_t::mode_t::ACTIVE;
        target_cmd_ptr->follow_yaw       = true;
        target_cmd_ptr->timestamp        = 0;
        target_cmd_ptr->vx               = rcdata_filter(rc.axes.lx * 3.0f);
        target_cmd_ptr->vy               = rcdata_filter(rc.axes.ly * 3.0f);
        target_cmd_ptr->wz               = 0.0f;
        target_cmd_ptr->wz2              = 0.0f;
        target_cmd_ptr->target_imu_delta = -rcdata_filter(rc.axes.rx)
                                         * IMU_RATE_SCALE * CTRL_DT;
    }
    else if (sw_pos_t::DOWN == sw_r)
    {
        // ACTIVE: 小陀螺模式 — 底盘固定速度自旋, 摇杆rx控制云台方向
        target_cmd_ptr->mode       = cmd_base_t::mode_t::ACTIVE;
        target_cmd_ptr->follow_yaw = false;
        target_cmd_ptr->timestamp  = 0;
        target_cmd_ptr->vx  = rcdata_filter(rc.axes.lx * 3.0f);
        target_cmd_ptr->vy  = rcdata_filter(rc.axes.ly * 3.0f);
        target_cmd_ptr->wz  = 2.0f;  // 固定自旋速度 rad/s
        target_cmd_ptr->wz2 = -rcdata_filter(rc.axes.rx)
                            * IMU_RATE_SCALE * CTRL_DT;  // rx控制云台朝向增量
    }

    // --- 左开关: 强制自旋 (优先级高于右开关的部分设定) ---
    if (sw_pos_t::MID == sw_l)
    {
        target_cmd_ptr->follow_yaw = false;
        target_cmd_ptr->wz         = 2.0f;
    }
    else if (sw_pos_t::DOWN == sw_l)
    {
        target_cmd_ptr->follow_yaw = false;
        target_cmd_ptr->wz         = -2.0f;
    }

    // --- IMU数据更新 (两种模式都需要: yaw电机统一用IMU位置PID) ---
    ins_drv->get_rads_b(
        &(target_robot_ptr->_yaw_data.current_insyaw),
        &(target_robot_ptr->_yaw_data.current_inspitch),
        &(target_robot_ptr->_yaw_data.current_insroll));

    // 进入跟随模式时: 用当前IMU角度锁定target_insyaw, 云台不跳变
    static bool was_following_prev = false;
    if (target_cmd_ptr->follow_yaw && !was_following_prev)
    {
        target_robot_ptr->_yaw_data.target_insyaw =
            target_robot_ptr->_yaw_data.current_insyaw;
    }
    was_following_prev = target_cmd_ptr->follow_yaw;
}

// ========== 依赖初始化 (电机 + PID) ==========
static void deps_init()
{
    target_deps_ptr = new target_deps_t();

    // ---------- 电机分配 ----------
    // CAN3: 舵机 + 轮子
    //   舵机0 (3号舵): GM6020, CAN3, ID=3
    target_deps_ptr->motor_deps.steering[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                     can_hub_t::can3);
    //   舵机1 (1号舵): GM6020, CAN3, ID=1
    target_deps_ptr->motor_deps.steering[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                     can_hub_t::can3);

    //   轮子0 (FL): M3508, CAN3, ID=2
    target_deps_ptr->motor_deps.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                   can_hub_t::can3);
    //   轮子1 (BL): M3508, CAN3, ID=4
    target_deps_ptr->motor_deps.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                   can_hub_t::can3);
    //   轮子2 (BR): M3508, CAN3, ID=3
    target_deps_ptr->motor_deps.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                   can_hub_t::can3);
    //   轮子3 (FR): M3508, CAN3, ID=1
    target_deps_ptr->motor_deps.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                   can_hub_t::can3);

    // CAN2: Yaw电机
    //   Yaw: GM6020, CAN2, ID=2
    target_deps_ptr->motor_deps.yaw =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                     can_hub_t::can2);

    // ---------- PID 分配 ----------
    // 轮子速度环 PID (4个)
    for (int i = 0; i < 4; i++)
        target_deps_ptr->pid_deps.wheel_pid[i] =
            new pid_t(0.2f, 0.1f, 0.00f, 1.00f, 20.0f);

    // 舵机0 位置环 + 速度环
    target_deps_ptr->pid_deps.steering_pos_pid[0] =
        new pid_t(20.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    target_deps_ptr->pid_deps.steering_spd_pid[0] =
        new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);

    // 舵机1 位置环 + 速度环
    target_deps_ptr->pid_deps.steering_pos_pid[1] =
        new pid_t(18.8f, 0.2f, 0.0f, 1.0f, 50.0f);
    target_deps_ptr->pid_deps.steering_spd_pid[1] =
        new pid_t(0.40f, 0.22f, 0.0f, 1.0f, 20.0f);

    // Yaw 位置环 + 速度环 (IMU PID, 两种模式共用)
    target_deps_ptr->pid_deps.yaw_pos_pid =
        new pid_t(32.8f, 0.1f, 0.0f, 1.0f, 50.0f);
    target_deps_ptr->pid_deps.yaw_spd_pid =
        new pid_t(0.1f, 0.00001f, 0.0000003f, 10.0f, 20.0f);

    // 底盘跟随yaw的PID (current_yaw_rad → 0 → wz)
    target_deps_ptr->pid_deps.follow_yaw_pid =
        new pid_t(2.0f, 0.01f, 0.05f, 1.0f, 2.0f);

    // ---------- 舵机零点偏移 ----------
    target_deps_ptr->steering_pos_offset[0] = -0.662679672f;
    target_deps_ptr->steering_pos_offset[1] = 2.95061207f;
    target_deps_ptr->steering_pos_offset[2] = -1.08145666f;
}

// =========================================================
// 外部C接口 (FreeRTOS任务入口)
// =========================================================
extern "C"
{

void target_chassis_thread(void *argument)
{
    while (true)
    {
        chassis_rc2cmd();
        target_robot_ptr->set_command(*target_cmd_ptr);
        vTaskDelay(1);
    }
}

void target_chassis_init(void *argument)
{
    // 获取 INS 实例
    ins_drv = ins_drv_t::get_instance();

    // 创建命令和依赖对象
    target_cmd_ptr   = new target_cmd_t();
    target_robot_ptr = target_robot_t::instance();

    // 初始化依赖并配置底盘模块
    deps_init();
    target_robot_ptr->configure(*target_deps_ptr);
    target_robot_ptr->start();

    // 创建遥控器命令处理线程
    xTaskCreate(target_chassis_thread, "target_chassis_thread", 512,
                nullptr, configMAX_PRIORITIES - 1, nullptr);

    vTaskDelete(nullptr);
}

} // extern "C"
