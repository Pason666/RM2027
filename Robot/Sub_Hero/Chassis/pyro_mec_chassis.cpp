#include "pyro_mec_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"


namespace pyro
{

// =========================================================
// 静态辅助函数
// =========================================================

// 角度/数值循环限幅 (处理 -PI 到 PI 的过零点)
static float _loop_fp32_constrain(float val, float min_val, float max_val)
{
    float len = max_val - min_val;
    if (len < 1e-6f)
        return val;
    while (val > max_val)
        val -= len;
    while (val < min_val)
        val += len;
    return val;
}

static float mps_to_rpm(const float mps, const float radius)
{
    if (radius < 1e-4f)
        return 0.0f;
    return (mps / radius) * 9.5492966f; // 60 / 2pi
}

static float _radps_to_rpm(const float radps)
{
    return radps * 9.5492966f;
}

// =========================================================
// 构造与初始化
// =========================================================

mec_chassis_t::mec_chassis_t() : module_base_t("mec_chassis")
{
    _ctx = {};
}

status_t mec_chassis_t::_init()
{
    _kinematics          = new mecanum_kin_t(WHEELBASE, TRACK_WIDTH);

    // 假设使用 M3508，CAN1
    _ctx.motor.wheels[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                                     can_hub_t::can1); // FL
    _ctx.motor.wheels[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                     can_hub_t::can1); // FR
    _ctx.motor.wheels[2] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                     can_hub_t::can1); // BL
    _ctx.motor.wheels[3] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                                     can_hub_t::can1); // BR

    // Yaw 轴电机 (GM6020)，CAN2 ID 2
    _ctx.motor.yaw_motor = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_2, can_hub_t::can2);

    // 轮组 PID (速度环)
    for (auto &pid : _ctx.pid.wheel_pid)
    {
        // pid = new pid_t(0.32f, 0.0003f, 0.0000f, 1.0f, 20.0f);
        pid = new pid_t(0.2f, 0.00f, 0.0000f, 1.0f, 20.0f);
    }

    // 跟随 PID (位置环：输入弧度误差，输出 rad/s)
    // 注意：P 参数可能需要根据底盘重量调整 (3.0 ~ 8.0)
    _ctx.pid.follow_pid = new pid_t(5.0f, 0.0f, 0.002f, 0.0f, 6.0f, 10, 10, 4);

    _ctx.powermeter     = new powermeter_drv_t(0x212, can_hub_t::can2);
    _ctx.powermeter->init();
    // 功率控制初始化
    _power_control_init();

    return PYRO_OK;
}

void mec_chassis_t::_power_control_init()
{
    power_control_drv_t::motor_coefficient_t coef[4];

    for (auto &[k1, k2, k3, k4] : coef)
    {
        // k1 = 0.0160f;//0.0155//0.0260
        // k2 = 0.0250f;//1.6000//0.0460
        // k3 = 0.1742f;//0.0010//0.1066
        // k4 = 0.5815f;//0.7500//0.7500
        k1 = 0.0260f; // 0.0155
        k2 = 0.0460f; // 1.6000
        k3 = 0.1100f; // 0.0010
        k4 = 0.7500f; // 0.7500
    }

    power_control_drv_t::get_instance(4).set_motor_coefficient(1, coef[0]);
    power_control_drv_t::get_instance(4).set_motor_coefficient(2, coef[1]);
    power_control_drv_t::get_instance(4).set_motor_coefficient(3, coef[2]);
    power_control_drv_t::get_instance(4).set_motor_coefficient(4, coef[3]);
}

void mec_chassis_t::_power_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.power_motor_data[i].gyro       = _ctx.data.current_wheel_rpm[i];
        _ctx.power_motor_data[i].torque_cmd = _ctx.data.out_wheel_torque[i];
        _ctx.power_motor_data[i].power_predict =
            power_control_drv_t::get_instance().motor_power_predict(
                i, _ctx.power_motor_data[i].torque_cmd,
                _ctx.power_motor_data[i].gyro);
    }
    if (_ctx.cap_feedback.vot_cap >= 1800)
    {
        power_control_drv_t::get_instance().calculate_restricted_torques(
            _ctx.power_motor_data, 4,
            static_cast<float>(referee_drv_t::get_instance()
                                   ->get_data()
                                   .robot_status.chassis_power_limit) +
                150.0f,buffer_engy);
    }
    else
    {
        power_control_drv_t::get_instance().calculate_restricted_torques(
        _ctx.power_motor_data, 4, referee_drv_t::get_instance()
                               ->get_data()
                               .robot_status.chassis_power_limit,buffer_engy);
        // power_control_drv_t::get_instance().calculate_restricted_torques(
        //     _ctx.power_motor_data, 4, 240);
    }
    for (int i = 0; i < 4; i++)
        _ctx.data.out_wheel_torque[i] =
            _ctx.power_motor_data[i].restricted_torque;
}


// =========================================================
// 核心循环回调
// =========================================================

void mec_chassis_t::_update_feedback()
{
    // 1. 更新电机底层数据
    for (auto *motor : _ctx.motor.wheels)
    {
        motor->update_feedback();
    }
    _ctx.motor.yaw_motor->update_feedback();

    // 2. 转换轮子数据 (RPM)
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            _radps_to_rpm(_ctx.motor.wheels[i]->get_current_rotate() *
                          dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.wheel_online[i] = _ctx.motor.wheels[i]->is_online();
    }

    // 3. 计算 Yaw 轴跟随误差 (关键修改)
    // -------------------------------------------------------------
    // 获取当前绝对角度 (减去机械零点偏移)
    float current_angle =
        _ctx.motor.yaw_motor->get_current_position() - YAW_OFFSET_RAD;

    // 归一化当前角度到 -PI ~ PI
    current_angle               = _loop_fp32_constrain(current_angle, -PI, PI);

    // 目标是让底盘对齐云台 (角度为0)
    // Error = Target - Current = 0 - Current
    float yaw_err               = current_angle;

    // 再次归一化 Error，确保走最短路径 (例如从 -3.1 到 +3.1 应该只差
    // 0.2，而不是 6.2)
    _ctx.data.current_yaw_error = _loop_fp32_constrain(yaw_err, -PI, PI);

    // 4. 更新 cap_tx 数据
    _ctx.supercap_cmd.power_referee = 0;
    _ctx.supercap_cmd.power_limit_referee =
        referee_drv_t::get_instance()
            ->get_data()
            .robot_status.chassis_power_limit;
    _ctx.supercap_cmd.power_buffer_limit_referee = 60.0f;
    _ctx.supercap_cmd.power_buffer_referee =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;
    _ctx.supercap_cmd.use_cap           = 1;
    _ctx.supercap_cmd.kill_chassis_user = 0;
    _ctx.supercap_cmd.speed_up_user_now = 0;

    buffer_engy =
        referee_drv_t::get_instance()->get_data().power_heat.buffer_energy;

    // 5. 更新 cap_rx 数据
    _ctx.cap_feedback = supercap_drv_t::get_instance()->get_feedback();

    _ctx.powermeter->get_data(_ctx.powermeter_feedback);
}

void mec_chassis_t::_kinematics_solve()
{
    // -------------------------------------------------------------
    // 1. 跟随 PID 计算 (算出底盘需要的自旋速度 wz)
    // -------------------------------------------------------------
    // Calculate(measurement, target) 或 (error, 0)
    // 假设 pid_t::calculate(target, current)，我们将 error 作为 P项输入
    float follow_wz =
        _ctx.pid.follow_pid->calculate(_ctx.data.current_yaw_error, 0.0f);

    // 最终角速度 = 跟随产生的角速度 + 选手手动输入的角速度(小陀螺/微调)
    float final_wz = follow_wz;


    // -------------------------------------------------------------
    // 2. 矢量旋转 (将云台坐标系速度转换到底盘坐标系)
    // -------------------------------------------------------------
    // 我们需要知道底盘相对于云台的角度。
    // current_yaw_error = 0 - current_angle => current_angle =
    // -current_yaw_error 设 theta 为底盘相对于云台的偏角
    float theta        = -_ctx.data.current_yaw_error;

    float c_theta      = cos(theta);
    float s_theta      = sin(theta);

    // 旋转矩阵公式 (逆时针旋转 theta)
    // V_chassis = RotationMatrix(-theta) * V_gimbal ?
    // 实际上我们需要把 云台系 投射到 底盘系。
    // 如果云台不动，底盘左转了 90度 (theta=90)，向前推杆
    // (vx=1)，底盘应该向右平移 (y=-1)。 vx_c = 1 * cos(90) + 0 = 0 vy_c = -1 *
    // sin(90) + 0 = -1 (符合)
    float vx_chassis   = _ctx.cmd->vx * c_theta + _ctx.cmd->vy * s_theta;
    float vy_chassis   = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy * c_theta;

    if (_ctx.cmd->wz != 0.0f)
    {
        final_wz = _ctx.cmd->wz;
    }
    else
    {
        if (abs(final_wz) > 3.0f)
        {
            vx_chassis *= 0.22f;
            vy_chassis *= 0.22f;
        }
    }

    int offline_count  = 0;
    auto missing_wheel = mecanum_kin_t::missing_mec_e::NONE;

    // 根据数组索引对应找出具体离线的轮子 (0:FL, 1:FR, 2:BL, 3:BR)
    if (!_ctx.data.wheel_online[0])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::FL;
    }
    if (!_ctx.data.wheel_online[1])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::FR;
    }
    if (!_ctx.data.wheel_online[2])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::BL;
    }
    if (!_ctx.data.wheel_online[3])
    {
        offline_count++;
        missing_wheel = mecanum_kin_t::missing_mec_e::BR;
    }

    // 如果有两个或以上的轮子离线，失去冗余控制能力，强制速度全为 0
    if (offline_count >= 2)
    {
        vx_chassis    = 0.0f;
        vy_chassis    = 0.0f;
        final_wz      = 0.0f;
        missing_wheel = mecanum_kin_t::missing_mec_e::NONE; // 速度全为0
    }

    // -------------------------------------------------------------
    // 4. 运动学解算 (带入缺失轮枚举)
    // -------------------------------------------------------------
    const auto wheel_speeds_mps =
        _kinematics->solve(vx_chassis, vy_chassis, final_wz, missing_wheel);

    // 4. 转 RPM 并分配给电机
    // 注意：右侧电机通常需要反转，取决于具体安装和电机库定义
    _ctx.data.target_wheel_rpm[0] =
        mps_to_rpm(wheel_speeds_mps.fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -mps_to_rpm(wheel_speeds_mps.fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        mps_to_rpm(wheel_speeds_mps.bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -mps_to_rpm(wheel_speeds_mps.br, WHEEL_RADIUS);
}

void mec_chassis_t::_chassis_control(mec_context_t *ctx)
{
    // 计算 4 个轮子的 PID
    for (int i = 0; i < 4; i++)
    {
        ctx->data.out_wheel_torque[i] = ctx->pid.wheel_pid[i]->calculate(
            ctx->data.target_wheel_rpm[i], ctx->data.current_wheel_rpm[i]);
        // if (ctx->data.target_wheel_rpm[i] < 0.001f)
        // {
        //     ctx->data.out_wheel_torque[i] = 0.0f;
        // }
    }
    _power_control();
    // ctx->data.out_wheel_torque[0] = 0;
    // ctx->data.out_wheel_torque[1] = 0;
    // ctx->data.out_wheel_torque[2] = 0;
    // ctx->data.out_wheel_torque[3] = 0;
}

void mec_chassis_t::_send_motor_command(mec_context_t *ctx)
{
    for (int i = 0; i < 4; i++)
    {
        ctx->motor.wheels[i]->send_torque(ctx->data.out_wheel_torque[i]);
        // ctx->motor.wheels[i]->send_torque(0);
    }
}

void mec_chassis_t::_send_supercap_command() const
{
    supercap_drv_t::get_instance()->send_cmd(_ctx.supercap_cmd); // NOLINT
}
// =========================================================
// 状态机逻辑
// =========================================================

void mec_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    static bool _last_status = false;
    static uint32_t _timer = 0;
    static bool _delay_done = false;

    bool current_status = referee_drv_t::get_instance()->get_data().robot_status.power_management_chassis_output;

    if (current_status)
    {
        // --- 情况 A：Chassis 有输出 ---
        if (!_last_status)
        {
            // 刚切到有输出状态：重置计时器和延迟标志
            _timer = 0;
            _delay_done = false;
        }

        if (!_delay_done)
        {
            // 1. 处理 1000 tick 的初始延迟
            if (++_timer >= 1000)
            {
                _delay_done = true;
                _timer = 0; // 重置用于后续的 10 tick 周期

                // 达到 1000 tick 时立即发送第一次开启指令
                _ctx.supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
        else
        {
            // 2. 延迟结束后，以 10 tick 为周期发送
            if (++_timer >= 10)
            {
                _timer = 0;
                _ctx.supercap_cmd.use_cap = 1;
                _send_supercap_command();
            }
        }
    }
    else
    {
        // --- 情况 B：Chassis 无输出 ---
        if (_last_status)
        {
            // 刚切换到无输出状态：发送一次 use_cap = 0
            _ctx.supercap_cmd.use_cap = 0;
            _send_supercap_command();

            // 重置状态位，防止重复发送
            _delay_done = false;
            _timer = 0;
        }
    }

    // 更新旧状态
    _last_status = current_status;

    _main_fsm.execute(this);
}

} // namespace pyro