#include "pyro_target_robot.h"
#include "pyro_core_def.h"
#include "pyro_module_base.h"
#include <arm_math.h>

namespace pyro
{

// =========================================================
// 构造函数
// =========================================================
target_robot_t::target_robot_t()
    : module_base_t("target", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data                         = {};
    // 初始化轮子方向: 全向轮0号正转=前进, 1号正转=前进
    _ctx.data.current_states.modules[0].direction = 1;
    _ctx.data.current_states.modules[1].direction = 1;
    // 舵轮方向初始化为-1
    _ctx.data.current_states.modules[2].direction = -1;
    _ctx.data.current_states.modules[3].direction = -1;
}

// =========================================================
// 初始化回调: 创建运动学模型，复制依赖到上下文
// =========================================================
status_t target_robot_t::_init()
{
    _ctx.motor            = _module_deps.motor_deps;
    _ctx.pid              = _module_deps.pid_deps;
    _ctx.steering_pos_offset[0] = _module_deps.steering_pos_offset[0];
    _ctx.steering_pos_offset[1] = _module_deps.steering_pos_offset[1];
    _ctx.steering_pos_offset[2] = _module_deps.steering_pos_offset[2];

    // 初始化运动学: 全向轮半径, 舵轮半径, dty1, dtx1, dty2, dtx2
    _kinematics = new rudomni_kin_t(
        TARGET_OMNI_WHEEL_RADIUS,  TARGET_RUD_WHEEL_RADIUS,
        TARGET_OMNI_DTY, TARGET_OMNI_DTX,
        TARGET_RUD_DTY,  TARGET_RUD_DTX);

    return PYRO_OK;
}

// =========================================================
// Yaw角度变换辅助函数（最短路径）
// =========================================================
void target_robot_t::yaw_angle_ch(float current_angle, float &target_angle)
{
    float angle_diff = target_angle - current_angle;

    while (angle_diff > PI)
        angle_diff -= 2.0f * PI;
    while (angle_diff < -PI)
        angle_diff += 2.0f * PI;

    if (fabsf(angle_diff) > PI / 2)
    {
        target_angle += PI;
        while (target_angle > PI)
            target_angle -= 2.0f * PI;
        while (target_angle < -PI)
            target_angle += 2.0f * PI;
    }

    angle_diff = target_angle - current_angle;

    while (angle_diff > PI)
        angle_diff -= 2.0f * PI;
    while (angle_diff < -PI)
        angle_diff += 2.0f * PI;

    target_angle = current_angle + angle_diff;
}

// =========================================================
// 传感器反馈更新
// =========================================================
void target_robot_t::_update_feedback()
{
    // 更新所有电机反馈
    _ctx.motor.steering[0]->update_feedback();
    _ctx.motor.steering[1]->update_feedback();
    _ctx.motor.wheel[0]->update_feedback();
    _ctx.motor.wheel[1]->update_feedback();
    _ctx.motor.wheel[2]->update_feedback();
    _ctx.motor.wheel[3]->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // 1. 两个舵机当前角度 (带零点偏移)
    _ctx.data.current_states.modules[2].angle =
        _ctx.motor.steering[0]->get_current_position()
        - _ctx.steering_pos_offset[0];
    _ctx.data.current_states.modules[3].angle =
        _ctx.motor.steering[1]->get_current_position()
        - _ctx.steering_pos_offset[1];

    // 角度归一化到 [-PI, PI]
    for (int i = 0; i < 2; i++)
    {
        if (_ctx.data.current_states.modules[i + 2].angle > PI)
            _ctx.data.current_states.modules[i + 2].angle -= 2 * PI;
        else if (_ctx.data.current_states.modules[i + 2].angle < -PI)
            _ctx.data.current_states.modules[i + 2].angle += 2 * PI;
    }

    // 2. 舵机当前角速度
    _ctx.data.current_steering_radps[0] =
        _ctx.motor.steering[0]->get_current_rotate();
    _ctx.data.current_steering_radps[1] =
        _ctx.motor.steering[1]->get_current_rotate();

    // 3. yaw电机当前位置和角速度
    _ctx.data.current_yaw_radps = _ctx.motor.yaw->get_current_rotate();
    _ctx.data.current_yaw_rad =
        _ctx.motor.yaw->get_current_position()
        - _ctx.steering_pos_offset[2];
    if (_ctx.data.current_yaw_rad > PI)
        _ctx.data.current_yaw_rad -= 2 * PI;
    else if (_ctx.data.current_yaw_rad < -PI)
        _ctx.data.current_yaw_rad += 2 * PI;

    // 4. 四个轮子的当前转速
    for (int i = 0; i < 4; i++)
        _ctx.data.current_states.modules[i].speed =
            _ctx.motor.wheel[i]->get_current_rotate();
}

// =========================================================
// 运动学解算
// =========================================================
void target_robot_t::_kinematics_solve()
{
    float vy2 = _ctx.cmd->vy, vx2 = _ctx.cmd->vx;

    // 将云台坐标系速度转换为底盘坐标系 (基于yaw电机当前位置)
    _ctx.cmd->vx =
        vx2 * cosf(_ctx.data.current_yaw_rad)
        - vy2 * sinf(_ctx.data.current_yaw_rad);
    _ctx.cmd->vy =
        vx2 * sinf(_ctx.data.current_yaw_rad)
        + vy2 * cosf(_ctx.data.current_yaw_rad);

    // IMU 目标角度累积 (两种模式共用target_insyaw, 来源不同)
    if (_ctx.cmd->follow_yaw == false)
    {
        // IMU锁头模式: wz2(摇杆ry) → target_insyaw 增量
        _yaw_data.target_insyaw += _ctx.cmd->wz2;
    }
    else
    {
        // 跟随模式: target_imu_delta(摇杆rx) → target_insyaw 增量
        _yaw_data.target_insyaw += _ctx.cmd->target_imu_delta;
    }
    yaw_angle_ch(_yaw_data.current_insyaw, _yaw_data.target_insyaw);

    // 跟随模式: 通过yaw电机角度差计算底盘wz, 驱动current_yaw_rad归零
    // follow_yaw_pid(0, current_yaw_rad) → error = -current_yaw_rad, 取反以正反馈归零
    if (_ctx.cmd->follow_yaw == true)
    {
        _ctx.cmd->wz = -_ctx.pid.follow_yaw_pid->calculate(
            0.0f, _ctx.data.current_yaw_rad);
    }

    // 调用运动学求解器
    _ctx.data.target_states = _kinematics->solve(
        _ctx.cmd->vx, _ctx.cmd->vy, _ctx.cmd->wz,
        _ctx.data.current_states);
}

// =========================================================
// 底盘控制 (PID计算)
// =========================================================
void target_robot_t::_chassis_control()
{
    // 舵机: 位置环 + 速度环
    for (int i = 0; i < 2; i++)
    {
        // 舵机位置环
        const float steering_pos_output =
            _ctx.pid.steering_pos_pid[i]->calculate(
                _ctx.data.target_states.modules[i + 2].angle,
                _ctx.data.current_states.modules[i + 2].angle);

        // 舵机速度环
        _ctx.data.out_steering_torque[i] =
            _ctx.pid.steering_spd_pid[i]->calculate(
                steering_pos_output,
                _ctx.data.current_steering_radps[i]);
    }

    // 全向轮: 速度环
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_wheel_torque[i] =
            _ctx.pid.wheel_pid[i]->calculate(
                _ctx.data.target_states.modules[i].speed,
                _ctx.data.current_states.modules[i].speed);
    }

    // Yaw轴控制: 两种模式统一使用IMU位置环 → 速度环
    // target_insyaw 在 _kinematics_solve 中由 wz2(IMU锁头) 或 target_imu_delta(跟随) 累积
    _ctx.data.target_yaw_rads =
        _ctx.pid.yaw_pos_pid->calculate(
            _yaw_data.target_insyaw,
            _yaw_data.current_insyaw);
    _ctx.data.out_yaw_torque =
        _ctx.pid.yaw_spd_pid->calculate(
            _ctx.data.target_yaw_rads,
            _ctx.data.current_yaw_radps);
}

// =========================================================
// 发送电机扭矩命令
// =========================================================
void target_robot_t::_send_motor_command() const
{
    // 舵机扭矩
    _ctx.motor.steering[0]->send_torque(_ctx.data.out_steering_torque[0]);
    _ctx.motor.steering[1]->send_torque(_ctx.data.out_steering_torque[1]);

    // 轮子扭矩
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.wheel[i]->send_torque(_ctx.data.out_wheel_torque[i]);
    }

    // Yaw电机扭矩
    _ctx.motor.yaw->send_torque(_ctx.data.out_yaw_torque);
}

// =========================================================
// 状态机执行 (每1ms调用一次)
// =========================================================
void target_robot_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro
