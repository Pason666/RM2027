#include "pyro_hybrid_chassis.h"
#include "pyro_algo_common.h"
#include <arm_math.h> // 引入 CMSIS-DSP 库
#include "pyro_dji_motor_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_power_control_drv.h"
#include "pyro_sr04_drv.h"

namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

hybrid_chassis_t::hybrid_chassis_t() : module_base_t("hybrid")
{
    _ctx = {};
}

status_t hybrid_chassis_t::_init()
{
    _ctx.motor  = _module_deps.motor_deps;
    _ctx.pid    = _module_deps.pid_deps;

    // 使用 config.h 中的参数初始化运动学模型
    _kinematics = new hybrid_kin_t(TRACK_SPACING,
                                   (MEC_FRONT_TRACK_WIDTH + MEC_WHEELBASE) / 2,
                                   (MEC_FRONT_TRACK_WIDTH + MEC_WHEELBASE) / 2,
                                   (MEC_REAR_TRACK_WIDTH + MEC_WHEELBASE) / 2,
                                   (MEC_REAR_TRACK_WIDTH + MEC_WHEELBASE) / 2);
    // float x = 0.15f;
    // _kinematics = new hybrid_kin_t(TRACK_SPACING,
    //                            0.15f + x,0.15f + x,0.66f - x, 0.66f - x);

    return PYRO_OK;
}

void hybrid_chassis_t::_power_control_init()
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


void hybrid_chassis_t::_update_feedback()
{
    // 1. 更新所有电机反馈
    for (const auto &i : _ctx.motor.mecanum)
        i->update_feedback();
    for (const auto &i : _ctx.motor.track)
        i->update_feedback();
    for (const auto &i : _ctx.motor.leg)
        i->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // // 2. 读取 IMU 数据作为底盘姿态反馈
    // ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_yaw_rad,
    //                                       &_ctx.data.current_pitch_rad,
    //                                       &_ctx.data.current_roll_rad);
    // _ctx.data.current_pitch_rad -= PITCH_OFFSET_RAD;
    // _ctx.data.current_roll_rad -= ROLL_OFFSET_RAD;
    // 2. 读取 IMU 数据作为底盘姿态反馈
    float raw_yaw, raw_pitch, raw_roll;
    ins_drv_t::get_instance()->get_rads_n(&raw_yaw, &raw_pitch, &raw_roll);

    // 减去机械安装零点偏移
    raw_pitch -= PITCH_OFFSET_RAD;
    raw_roll -= ROLL_OFFSET_RAD;

    // --- 一阶低通滤波 (LPF) ---
    // 为了快速验证，这里使用 static 变量保存上一次的滤波状态
    // 如果确认有效，建议将它们移到 _ctx.data 结构体中
    static float filtered_pitch = 0.0f;
    static float filtered_roll  = 0.0f;
    static bool is_first_run    = true;

    // 滤波系数 alpha：(0, 1]
    // alpha = 1.0 表示完全不滤波；alpha 越小，抗噪声能力越强，但相位延迟越大。
    // 对于 500Hz~1000Hz 的控制循环，0.1f ~ 0.3f 通常是一个比较理想的甜点值。
    const float LPF_ALPHA       = 0.15f;

    if (is_first_run)
    {
        // 第一次运行直接赋值，防止开机瞬间出现从 0 平滑过去的巨大阶跃
        filtered_pitch = raw_pitch;
        filtered_roll  = raw_roll;
        is_first_run   = false;
    }
    else
    {
        // 迭代滤波公式
        filtered_pitch =
            LPF_ALPHA * raw_pitch + (1.0f - LPF_ALPHA) * filtered_pitch;
        filtered_roll =
            LPF_ALPHA * raw_roll + (1.0f - LPF_ALPHA) * filtered_roll;
    }

    // 将滤波后的平滑数据赋给上下文，供 VMC 和 PID 使用
    _ctx.data.current_yaw_rad   = raw_yaw; // Yaw 通常不参与重力补偿，可暂不滤波
    _ctx.data.current_pitch_rad = filtered_pitch;
    _ctx.data.current_roll_rad  = filtered_roll;
    _ctx.data.distance_mm       = sr04_drv::get_instance().get_distance();

    // 3. 转换并记录电机转速与位置
    float current_angle =
        _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    current_angle = loop_fp32_constrain(current_angle, -PI, PI);
    _ctx.data.current_yaw_error =
        loop_fp32_constrain(0 - current_angle, -PI, PI);

    for (int i = 0; i < 4; i++)
    {
        _ctx.data.current_wheel_rpm[i] =
            radps_to_rpm(_ctx.motor.mecanum[i]->get_current_rotate() *
                         dji_m3508_motor_drv_t::reciprocal_reduction_ratio);
        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    auto real_vel = _kinematics->forward_solve(
        rpm_to_mps(_ctx.data.current_wheel_rpm[0], WHEEL_RADIUS),
        rpm_to_mps(-_ctx.data.current_wheel_rpm[1], WHEEL_RADIUS),
        rpm_to_mps(_ctx.data.current_wheel_rpm[2], WHEEL_RADIUS),
        rpm_to_mps(-_ctx.data.current_wheel_rpm[3], WHEEL_RADIUS));

    _ctx.data.real_vx = real_vel.vx;
    _ctx.data.real_vy = real_vel.vy;
    _ctx.data.real_wz = real_vel.wz;

    for (int i = 0; i < 2; i++)
        _ctx.data.current_track_rpm[i] =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());

    // 左右腿对称性修正：对右腿(leg[1])的读取数据取反，抹平机械差异
    // 左右腿对称性修正与机械零点 Offset 处理
    // 【左腿】：原本是 (pos - offset)，现在电机反转，所以整体取反变成 -(pos -
    // offset)
    float left_leg_raw =
        -(_ctx.motor.leg[0]->get_current_position() - LEFT_LEG_OFFSET_RAD);
    _ctx.data.current_leg_rad[0]   = loop_fp32_constrain(left_leg_raw, -PI, PI);
    _ctx.data.current_leg_radps[0] = -_ctx.motor.leg[0]->get_current_rotate();

    // 【右腿】：原本就是 -(pos - offset)，现在电机也反了，负负得正变成 (pos -
    // offset)
    float right_leg_raw =
        _ctx.motor.leg[1]->get_current_position() - RIGHT_LEG_OFFSET_RAD;
    _ctx.data.current_leg_rad[1] = loop_fp32_constrain(right_leg_raw, -PI, PI);
    _ctx.data.current_leg_radps[1] = _ctx.motor.leg[1]->get_current_rotate();
}

// =========================================================
// 核心解算与控制逻辑
// =========================================================

void hybrid_chassis_t::_kinematics_solve()
{
    // -------------------------------------------------------------
    // 1. 跟随 PID 计算 (算出底盘需要的自旋速度 wz)
    // -------------------------------------------------------------
    // Calculate(measurement, target) 或 (error, 0)
    // 假设 pid_t::calculate(target, current)，我们将 error 作为 P项输入
    float yaw_err = _ctx.data.target_yaw_rad - _ctx.data.current_yaw_rad;
    if (yaw_err < -PI)
    {
        _ctx.data.target_yaw_rad += 2 * PI;
    }
    else if (yaw_err > PI)
    {
        _ctx.data.target_yaw_rad -= 2 * PI;
    }
    // const float follow_wz = _ctx.pid.follow_yaw_pid->calculate(
    //     _ctx.data.target_yaw_rad, _ctx.data.current_yaw_rad);

    const float follow_wz =
        _ctx.pid.follow_yaw_pid->calculate(0.0f, _ctx.data.current_yaw_error);

    // 最终角速度 = 跟随产生的角速度 + 选手手动输入的角速度(小陀螺/微调)
    float final_wz          = follow_wz;

    // -------------------------------------------------------------
    // 2. 矢量旋转 (将云台坐标系速度转换到底盘坐标系)
    // const float theta       = _ctx.data.current_yaw_error;
    //
    // const float c_theta     = arm_cos_f32(theta);
    // const float s_theta     = arm_sin_f32(theta);
    //
    // // 旋转矩阵公式 (逆时针旋转 theta)
    // const float vx_chassis  = _ctx.cmd->vx * c_theta + _ctx.cmd->vy *
    // s_theta; const float vy_chassis  = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy
    // * c_theta;
    //
    // const auto wheel_speeds = _kinematics->solve(vx_chassis, vy_chassis,
    //                                              final_wz,
    //                                              _ctx.cmd->track_en);

    const float theta       = _ctx.data.current_yaw_error;
    // const float theta   = 0;

    const float c_theta     = arm_cos_f32(theta);
    const float s_theta     = arm_sin_f32(theta);

    // 旋转矩阵公式 (逆时针旋转 theta)
    float vx_chassis        = _ctx.cmd->vx * c_theta + _ctx.cmd->vy * s_theta;
    float vy_chassis        = -_ctx.cmd->vx * s_theta + _ctx.cmd->vy * c_theta;



    int offline_count       = 0;
    auto missing_wheel      = hybrid_kin_t::missing_mec_e::NONE;

    // // 根据数组索引对应找出具体离线的轮子 (0:FL, 1:FR, 2:BL, 3:BR)
    // if (!_ctx.data.wheel_online[0])
    // {
    //     offline_count++;
    //     missing_wheel = hybrid_kin_t::missing_mec_e::FL;
    // }
    // if (!_ctx.data.wheel_online[1])
    // {
    //     offline_count++;
    //     missing_wheel = hybrid_kin_t::missing_mec_e::FR;
    // }
    // if (!_ctx.data.wheel_online[2])
    // {
    //     offline_count++;
    //     missing_wheel = hybrid_kin_t::missing_mec_e::BL;
    // }
    // if (!_ctx.data.wheel_online[3])
    // {
    //     offline_count++;
    //     missing_wheel = hybrid_kin_t::missing_mec_e::BR;
    // }
    //
    // // 如果有两个或以上的轮子离线，失去冗余控制能力，强制速度全为 0
    // if (offline_count >= 2)
    // {
    //     vx_chassis    = 0.0f;
    //     vy_chassis    = 0.0f;
    //     final_wz      = 0.0f;
    //     missing_wheel = hybrid_kin_t::missing_mec_e::NONE; // 速度全为0
    // }

    // -------------------------------------------------------------
    // 4. 运动学解算 (带入缺失轮枚举)
    // -------------------------------------------------------------
    const auto wheel_speeds = _kinematics->solve(
        vx_chassis, vy_chassis, final_wz, _ctx.cmd->crossing_en, missing_wheel);

    // 麦轮转速分配 (右侧反转视底层驱动而定，此处按常规处理)
    _ctx.data.target_wheel_rpm[0] =
        mps_to_rpm(wheel_speeds.mec_fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] =
        -mps_to_rpm(wheel_speeds.mec_fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] =
        mps_to_rpm(wheel_speeds.mec_bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] =
        -mps_to_rpm(wheel_speeds.mec_br, WHEEL_RADIUS);

    // 履带分配 (差速模型)
    if (_ctx.cmd->crossing_en)
    {
        _ctx.data.target_track_rpm[0] =
            mps_to_rpm(wheel_speeds.track_l, TRACK_RADIUS);
        _ctx.data.target_track_rpm[1] =
            -mps_to_rpm(wheel_speeds.track_r, TRACK_RADIUS);
    }
    else
    {
        _ctx.data.target_track_rpm[0] = 0.0f;
        _ctx.data.target_track_rpm[1] = 0.0f;
    }

    _ctx.data.target_pitch_rad = NORMAL_PITCH;
}

void hybrid_chassis_t::_power_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.power_motor_data[i].gyro       = _ctx.data.current_wheel_rpm[i];
        _ctx.power_motor_data[i].torque_cmd = _ctx.data.out_mecanum_torque[i];
        _ctx.power_motor_data[i].power_predict =
            power_control_drv_t::get_instance().motor_power_predict(
                i, _ctx.power_motor_data[i].torque_cmd,
                _ctx.power_motor_data[i].gyro);
    }
    // if (_ctx.cap_feedback.vot_cap >= 1800)
    // {
    //     power_control_drv_t::get_instance().calculate_restricted_torques(
    //         _ctx.power_motor_data, 4,
    //         static_cast<float>(referee_drv_t::get_instance()
    //                                ->get_data()
    //                                .robot_status.chassis_power_limit) +
    //             100.0f);
    // }
    // else
    // {
    power_control_drv_t::get_instance().calculate_restricted_torques(
        _ctx.power_motor_data, 4, 240, 60);
    // }
    for (int i = 0; i < 4; i++)
        _ctx.data.out_mecanum_torque[i] =
            _ctx.power_motor_data[i].restricted_torque;
}


void hybrid_chassis_t::_leg_vmc()
{
    const float pitch     = _ctx.data.current_pitch_rad;
    const float roll      = _ctx.data.current_roll_rad;

    // 预计算 DSP 三角函数
    const float cos_pitch = arm_cos_f32(pitch);
    const float sin_pitch = arm_sin_f32(pitch);

    // 1. 计算姿态维稳所需的宏观虚拟力
    const float f_pitch =
        _ctx.pid.pitch_pid->calculate(_ctx.data.target_pitch_rad, pitch);
    const float f_roll = _ctx.pid.roll_pid->calculate(0, roll);

    for (int i = 0; i < 2; i++)
    {
        const float theta     = _ctx.data.current_leg_rad[i];
        const float theta_dot = _ctx.data.current_leg_radps[i];

        // 2. 多项式求解雅可比及端点坐标
        const float j_x =
            evaluate_polynomial(theta, JX_POLY_COEF, JX_POLY_DEGREE);
        const float j_y =
            evaluate_polynomial(theta, JY_POLY_COEF, JY_POLY_DEGREE);
        const float x_b =
            evaluate_polynomial(theta, XB_POLY_COEF, XB_POLY_DEGREE);
        const float y_b =
            evaluate_polynomial(theta, YB_POLY_COEF, YB_POLY_DEGREE);

        // 3. 计算重力前馈补偿 (使用预计算的 DSP 三角函数)
        const float y_wheel   = y_b - H_HIP_OFFSET;
        const float numerator = DIST_FRONT * cos_pitch - H_COG * sin_pitch;
        const float denominator =
            (DIST_FRONT + DIST_HIP + x_b) * cos_pitch + y_wheel * sin_pitch;

        float f_gravity_ff = 0.0f;
        if (fabsf(denominator) > 1e-4f)
        {
            f_gravity_ff = (MASS * GRAVITY * numerator) / denominator;
        }

        // 4. 提取动态雅可比标量
        const float j_dynamic   = j_y * cos_pitch + j_x * sin_pitch;

        // 5. 将任务空间的力分别映射为关节空间的力矩
        const float tau_gravity = j_dynamic * (0.5f * f_gravity_ff);
        float tau_pid  = j_dynamic * (f_pitch + (i == 0 ? f_roll : -f_roll));

        // 6. 虚拟阻尼墙限位保护
        float tau_wall = 0.0f;
        if (theta > LEG_MAX_POS - LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                -LEG_K_WALL * (theta - (LEG_MAX_POS - LEG_POS_BUFFER_RAD)) -
                LEG_D_WALL * theta_dot;
            tau_wall = fminf(0.0f, tau_wall);
        }
        else if (theta < LEG_MIN_POS + LEG_POS_BUFFER_RAD)
        {
            tau_wall =
                LEG_K_WALL * ((LEG_MIN_POS + LEG_POS_BUFFER_RAD) - theta) -
                LEG_D_WALL * theta_dot;
            tau_wall = fmaxf(0.0f, tau_wall);
        }

        // 7. 力矩饱和安全限制 (基于优先级的削峰逻辑)
        const float tau_priority = tau_gravity + tau_wall;
        float tau_total          = 0.0f;

        if (fabsf(tau_priority + tau_pid) > LEG_MAX_TORQUE)
        {
            if (fabsf(tau_priority) >= LEG_MAX_TORQUE)
            {
                tau_total =
                    (tau_priority > 0.0f) ? LEG_MAX_TORQUE : -LEG_MAX_TORQUE;
            }
            else
            {
                const float tau_avail = LEG_MAX_TORQUE - fabsf(tau_priority);
                tau_pid   = (tau_pid > 0.0f) ? tau_avail : -tau_avail;
                tau_total = tau_priority + tau_pid;
            }
        }
        else
        {
            tau_total = tau_priority + tau_pid;
        }

        tau_total = fminf(fmaxf(tau_total, -LEG_MAX_TORQUE), LEG_MAX_TORQUE);

        // tau_total = tau_gravity; // 重力补偿测试
        // if (fabsf(tau_total) > LEG_MAX_TORQUE)
        // {
        //     tau_total =
        //             (tau_total > 0.0f) ? LEG_MAX_TORQUE : -LEG_MAX_TORQUE;
        // }
        // 8. 输出并再次针对右腿作符号映射
        // _ctx.data.out_leg_torque[i] = (i == 0 ? 1.0f : -1.0f) * tau_gravity;
        _ctx.data.out_leg_torque[i] = (i == 0 ? -1.0f : 1.0f) * tau_total;
    }
}

void hybrid_chassis_t::_leg_length_control()
{
    const float pitch        = _ctx.data.current_pitch_rad;

    // 预计算 DSP 三角函数，用于雅可比映射与重力场旋转投影
    const float cos_pitch    = arm_cos_f32(pitch);
    const float sin_pitch    = arm_sin_f32(pitch);

    // 1. 预计算收腿的目标长度 (Task Space Target)
    // 使用专为长度控制放宽的限幅 LEG_LENGTH_MIN_POS (0.0f) 加上缓冲，
    // 确保收腿指令能引导机构钻入最深处的物理限位
    const float target_theta = LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD;
    const float target_y =
        evaluate_polynomial(target_theta, YB_POLY_COEF, YB_POLY_DEGREE);

    for (int i = 0; i < 2; i++)
    {
        const float theta     = _ctx.data.current_leg_rad[i];
        const float theta_dot = _ctx.data.current_leg_radps[i];

        // 2. 正向运动学：计算当前真实腿长 y_b 与动态雅可比
        const float j_x =
            evaluate_polynomial(theta, JX_POLY_COEF, JX_POLY_DEGREE);
        const float j_y =
            evaluate_polynomial(theta, JY_POLY_COEF, JY_POLY_DEGREE);
        const float y_b =
            evaluate_polynomial(theta, YB_POLY_COEF, YB_POLY_DEGREE);

        // 当前腿部在 Y 轴的真实收缩线速度 (m/s)
        const float y_dot = j_y * theta_dot;

        // 3. 任务空间(腿长)串级 PID 计算期望动态拉力 f_pid (N)
        _ctx.data.target_leg_radps[i] =
            _ctx.pid.leg_pos_pid[i]->calculate(target_y, y_b);
        float f_pid = _ctx.pid.leg_vel_pid[i]->calculate(
            _ctx.data.target_leg_radps[i], y_dot);

        // 4. 雅可比映射：将直线拉力转换为电机主动力矩 (仅为运动所需力)
        const float j_dynamic = j_y * cos_pitch + j_x * sin_pitch;
        float tau_pid         = j_dynamic * f_pid;

        // ==========================================================
        // 5. 【核心】：非线性自重补偿 + 旋转场降维投影
        // ==========================================================
        // 从 SolidWorks 提取的纯粹抗重力拟合多项式
        float tau_gravity_base =
            evaluate_polynomial(theta, TAU_GRAVITY_COEF, TAU_GRAVITY_DEGREE);

        // 将平地重力映射乘上 cos(pitch)，衰减掉倾斜失去的垂直分量
        // 同时引入你在 config 中预留的微调系数 K_TAU_GRAVITY
        float tau_gravity_ff = tau_gravity_base * cos_pitch * K_TAU_GRAVITY;

        // ==========================================================
        // 6. 极软虚拟墙保护与 PID 强制剥权 (基于专用的长度限幅)
        // ==========================================================
        float tau_wall       = 0.0f;

        // 使用针对悬空状态设计的极软参数 (LEG_GRA_K_WALL / LEG_GRA_D_WALL)
        // 限幅基准替换为 LEG_LENGTH_MAX_POS 和 LEG_LENGTH_MIN_POS
        if (theta > LEG_LENGTH_MAX_POS - LEG_LENGTH_POS_BUFFER_RAD)
        {
            const float spring =
                -LEG_GRA_K_WALL *
                (theta - (LEG_LENGTH_MAX_POS - LEG_LENGTH_POS_BUFFER_RAD));
            const float damp =
                (theta_dot > 0.0f) ? (-LEG_GRA_D_WALL * theta_dot) : 0.0f;
            tau_wall = fminf(0.0f, spring + damp);

            // 剥权保护：方向错误时没收 PID 控制权
            if (tau_pid > 0.0f)
                tau_pid = 0.0f;
        }
        else if (theta < LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD)
        {
            const float spring =
                LEG_GRA_K_WALL *
                ((LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD) - theta);
            const float damp =
                (theta_dot < 0.0f) ? (-LEG_GRA_D_WALL * theta_dot) : 0.0f;
            tau_wall = fmaxf(0.0f, spring + damp);

            // 剥权保护：方向错误时没收 PID 控制权
            if (tau_pid < 0.0f)
                tau_pid = 0.0f;
        }

        // 7. 物理限幅与最终输出
        float tau_total             = tau_pid + tau_wall + tau_gravity_ff;
        // float tau_total = tau_gravity_ff;
        tau_total                   = fminf(fmaxf(tau_total, -8.0f), 8.0f);

        // 针对右腿作符号反转映射
        // _ctx.data.out_leg_torque[i] = (i == 0 ? 1.0f : -1.0f) * tau_total;
        _ctx.data.out_leg_torque[i] = (i == 0 ? -1.0f : 1.0f) * tau_total;
    }
}

void hybrid_chassis_t::_communicate_gimbal() const
{
    pyro::can_tx_drv_t::clear(0x102);
    pyro::can_tx_drv_t::add_data(0x102, 32, _ctx.data.current_pitch_rad);
    pyro::can_tx_drv_t::send(
        0x102, can_hub_t::get_instance()->hub_get_can_obj(can_hub_t::can1));
}



void hybrid_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_mecanum_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
    // _power_control();
    // _ctx.data.out_mecanum_torque[0] = 0;
    //  _ctx.data.out_mecanum_torque[1] = 0;
    //  // _ctx.data.out_mecanum_torque[2] = 0;
    //  _ctx.data.out_mecanum_torque[3] = 0;
}

void hybrid_chassis_t::_track_control()
{
    for (int i = 0; i < 2; i++)
    {
        _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
            _ctx.data.target_track_rpm[i], _ctx.data.current_track_rpm[i]);
    }
    // _ctx.data.out_track_torque[0] = 0;
    // _ctx.data.out_track_torque[1] = 0;
}

void hybrid_chassis_t::_send_motor_command() const
{
    // 静态分频标志位，每次调用翻转一次，实现 1/2 频率
    static bool freq_div_flag = false;
    freq_div_flag             = !freq_div_flag;

    // 麦轮和履带：仅在 flag 为 true 时发送指令
    if (freq_div_flag)
    {
        for (int i = 0; i < 4; i++)
            _ctx.motor.mecanum[i]->send_torque(_ctx.data.out_mecanum_torque[i]);
        // _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_mecanum_torque[0]);
        // _ctx.motor.mecanum[1]->send_torque(0);
        // _ctx.motor.mecanum[2]->send_torque(0);
        // _ctx.motor.mecanum[3]->send_torque(0);

        for (int i = 0; i < 2; i++)
            _ctx.motor.track[i]->send_torque(_ctx.data.out_track_torque[i]);
        // for (int i = 0; i < 4; i++)
        //     _ctx.motor.mecanum[i]->send_torque(0);
        // for (int i = 0; i < 2; i++)
        //     _ctx.motor.track[i]->send_torque(0);
    }
    // 腿部电机：保持原频率控制 (VMC 和腿长控制通常需要高频以维持稳定性)
    for (int i = 0; i < 2; i++)
        _ctx.motor.leg[i]->send_torque(_ctx.data.out_leg_torque[i]);
}
// =========================================================
// 核心运行时与状态机
// =========================================================

void hybrid_chassis_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);
    else if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);

    _main_fsm.execute(this);
}


} // namespace pyro