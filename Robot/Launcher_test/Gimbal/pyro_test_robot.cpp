#include "pyro_test_robot.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include <arm_math.h>
#include <cmath>

float pitch_position;

namespace pyro
{

// --- 辅助: 将角度包裹到 [-PI, PI] ---
static inline float wrap_pi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;
    while (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}

// =========================================================
// 构造函数
// =========================================================
test_robot_t::test_robot_t()
    : module_base_t("test_robot", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data = {};
}

// =========================================================
// 初始化回调: 将依赖复制到上下文
// =========================================================
status_t test_robot_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    return PYRO_OK;
}

// =========================================================
// 传感器反馈更新
//   拨弹盘使用多圈连续角度追踪 (参考17mm实现)
// =========================================================
void test_robot_t::_update_feedback()
{
    // 更新所有电机反馈
    _ctx.motor.pitch->update_feedback();
    _ctx.motor.friction[0]->update_feedback();
    _ctx.motor.friction[1]->update_feedback();
    _ctx.motor.feeder->update_feedback();

    // 1. Pitch轴当前位置和角速度 (无零点偏移)
    _ctx.data.current_pitch_radps = _ctx.motor.pitch->get_current_rotate();
    _ctx.data.current_pitch_rad = _ctx.motor.pitch->get_current_position();

    pitch_position = _ctx.data.current_pitch_rad;

    // 2. 摩擦轮当前角速度
    _ctx.data.current_friction_radps[0] =
        _ctx.motor.friction[0]->get_current_rotate();
    _ctx.data.current_friction_radps[1] =
        _ctx.motor.friction[1]->get_current_rotate();

    // 3. 拨弹盘: 多圈连续角度追踪
    float rotor_rad = _ctx.motor.feeder->get_current_position();

    if (_ctx.data.feeder_first_update)
    {
        // 首次初始化: 以当前编码器位置为基准
        _ctx.data.feeder_last_rotor_rad = rotor_rad;
        _ctx.data.current_feeder_rad    = rotor_rad / TRIGGER_GEAR_RATIO;
        _ctx.data.trigger_offset        = _ctx.data.current_feeder_rad;
        _ctx.data.feeder_first_update   = false;
    }
    else
    {
        // 计算转子增量, 处理跨圈边界
        float delta_rad = rotor_rad - _ctx.data.feeder_last_rotor_rad;
        delta_rad       = wrap_pi(delta_rad);  // 跨 PI 时相位展开

        // 增量除以减速比, 累加到连续的拨弹盘世界角度中
        _ctx.data.current_feeder_rad += delta_rad / TRIGGER_GEAR_RATIO;
        _ctx.data.feeder_last_rotor_rad = rotor_rad;
    }

    // 拨弹盘角速度 (已除以减速比)
    _ctx.data.current_feeder_radps =
        _ctx.motor.feeder->get_current_rotate() / TRIGGER_GEAR_RATIO;
}

// =========================================================
// 云台控制 (Pitch 单轴 位置环 → 速度环)
//   摇杆非零: 积分目标位置 → PID跟踪
//   摇杆归零: PID锁位 (位置环锁当前目标不动)
// =========================================================
void test_robot_t::_gimbal_control()
{
    constexpr float CTRL_DT = 0.001f; // 1ms

    // --- Pitch轴: 摇杆角速度 → 位置积分 → 限幅 → 位置环 → 速度环 ---
    _ctx.data.target_pitch_rad += _ctx.cmd->pitch_rate * CTRL_DT;

    // 限制pitch目标位置在允许范围内
    if (_ctx.data.target_pitch_rad < PITCH_MIN_LIMIT)
        _ctx.data.target_pitch_rad = PITCH_MIN_LIMIT;
    if (_ctx.data.target_pitch_rad > PITCH_MAX_LIMIT)
        _ctx.data.target_pitch_rad = PITCH_MAX_LIMIT;

    _ctx.data.target_pitch_radps =
        _ctx.pid.pitch_pos_pid->calculate(_ctx.data.target_pitch_rad,
                                           _ctx.data.current_pitch_rad);
    _ctx.data.out_pitch_torque =
        _ctx.pid.pitch_spd_pid->calculate(_ctx.data.target_pitch_radps,
                                           _ctx.data.current_pitch_radps);
}

// =========================================================
// 摩擦轮控制 (两阶段停机, 参考17mm STOP状态)
//   阶段1: 目标转速非零 → PID速度环正常输出
//   阶段2: 目标转速=0 + PID刹车到低速 → 切换零力矩, PID失能
// =========================================================
void test_robot_t::_friction_control()
{
    if (_ctx.cmd->friction_speed != 0.0f)
    {
        // 目标非零: 重新激活PID
        _ctx.data.fric_pid_active = true;
    }

    if (_ctx.data.fric_pid_active)
    {
        // PID 速度环 (含主动刹车)
        _ctx.data.out_friction_torque[0] =
            _ctx.pid.friction_spd_pid[0]->calculate(
                -_ctx.cmd->friction_speed,
                _ctx.data.current_friction_radps[0]);

        _ctx.data.out_friction_torque[1] =
            _ctx.pid.friction_spd_pid[1]->calculate(
                _ctx.cmd->friction_speed,
                _ctx.data.current_friction_radps[1]);

        // 目标为零且两轮均已停转 → 切换到零力矩
        if (_ctx.cmd->friction_speed == 0.0f &&
            std::abs(_ctx.data.current_friction_radps[0]) < TEST_FRIC_STOP_THRESHOLD &&
            std::abs(_ctx.data.current_friction_radps[1]) < TEST_FRIC_STOP_THRESHOLD)
        {
            _ctx.data.fric_pid_active = false;
            for (int i = 0; i < 2; i++)
            {
                _ctx.pid.friction_spd_pid[i]->clear();
                _ctx.data.out_friction_torque[i] = 0.0f;
            }
        }
    }
    else
    {
        // PID 失能: 纯零力矩
        for (int i = 0; i < 2; i++)
            _ctx.data.out_friction_torque[i] = 0.0f;
    }
}

// =========================================================
// 拨弹盘子状态切换
// =========================================================
void test_robot_t::_trig_goto(decltype(test_robot_data_ctx_t::trig_state) s)
{
    using s_t = test_robot_data_ctx_t::trig_state_e;

    // 退出旧状态 → 清除卡弹计时
    _ctx.data.jam_start_tick = 0;

    // 进入新状态
    switch (s)
    {
    case s_t::IDLE:
        // 空闲状态: 位置环锁位
        _ctx.pid.feeder_pos_pid->clear();
        _ctx.pid.feeder_spd_pid->clear();
        _ctx.data.target_feeder_rad = _ctx.data.current_feeder_rad;
        _ctx.data.target_feeder_radps = 0;
        break;

    case s_t::INIT_FORWARD:
    {
        // 初始化正转: 速度环正转直至卡住
        _ctx.data.jam_start_tick = 0;
        _ctx.pid.feeder_spd_pid->clear();
        _ctx.data.target_feeder_radps = INIT_FORWARD_RADPS;
        break;
    }

    case s_t::READY:
        _trig_ready_enter();
        break;

    case s_t::DONE:
        // DONE 状态: 位置环锁位
        _ctx.pid.feeder_pos_pid->clear();
        _ctx.pid.feeder_spd_pid->clear();
        _ctx.data.target_feeder_rad = _ctx.data.current_feeder_rad;
        _ctx.data.target_feeder_radps = 0;
        break;

    case s_t::SINGLE:
    {
        // 单发: 从当前目标位置再推进一个槽位
        _ctx.pid.feeder_pos_pid->clear();
        _ctx.pid.feeder_spd_pid->clear();

        // 基于当前目标位置（而非当前实际位置）计算下一个槽位
        // 这样可以避免累积误差和计算错误
        float relative_target = _ctx.data.target_feeder_rad - _ctx.data.trigger_offset;
        // 计算目标在第几个槽位（向上取整，确保到达下一个完整槽位）
        int32_t current_target_slot = static_cast<int32_t>(std::ceil(relative_target / SINGLE_SHOT_ANGLE));
        // 下一个槽位
        int32_t next_slot = current_target_slot + 1;
        // 计算新的目标位置
        _ctx.data.target_feeder_rad = _ctx.data.trigger_offset + next_slot * SINGLE_SHOT_ANGLE;

        _ctx.data.target_feeder_radps = 0;
        _ctx.data.jam_start_tick    = 0;
        _ctx.data.jam_recovery_count = 0;  // 重置卡弹恢复计数
        _ctx.data.single_start_tick   = xTaskGetTickCount();
        break;
    }

    case s_t::CONTINUE:
    {
        // 连发: 速度环
        // 注意: 不清除速度环PID积分，避免重新建立积分造成卡顿
        if (_ctx.data.trig_state != s_t::CONTINUE &&
            _ctx.data.trig_state != s_t::JAM_RECOVERY)
        {
            _ctx.pid.feeder_spd_pid->clear();
        }
        _ctx.data.target_feeder_radps = _ctx.cmd->feeder_speed;
        _ctx.data.jam_start_tick    = 0;
        break;
    }

    case s_t::JAM_RECOVERY:
    {
        // 卡弹恢复: 无力状态
        _ctx.data.jam_recovery_start_tick = xTaskGetTickCount();
        _ctx.data.target_feeder_radps = 0;
        break;
    }
    }

    _ctx.data.trig_state = s;
}

// =========================================================
// 拨弹盘子状态: IDLE 执行
//   等待初始化触发信号
// =========================================================
void test_robot_t::_trig_idle_execute()
{
    // 位置环锁位
    _ctx.data.target_feeder_rad = _ctx.data.current_feeder_rad;
    _ctx.data.target_feeder_radps = 0;

    // 等待初始化触发 (feeder_trigger信号)
    bool trigger = _ctx.cmd->feeder_trigger && !_ctx.data.cmd_feeder_trigger;
    _ctx.data.cmd_feeder_trigger = _ctx.cmd->feeder_trigger;

    if (trigger)
    {
        // 触发初始化: 正转找零点
        _trig_goto(test_robot_data_ctx_t::trig_state_e::INIT_FORWARD);
    }
}

// =========================================================
// 拨弹盘子状态: INIT_FORWARD 执行
//   正转直到卡住, 记录零点, 打开摩擦轮
// =========================================================
void test_robot_t::_trig_init_forward_execute()
{
    uint32_t now = xTaskGetTickCount();

    // 维持正转速度
    _ctx.data.target_feeder_radps = INIT_FORWARD_RADPS;

    // --- 卡住检测: 实际速度低于阈值且持续一段时间 ---
    if (std::abs(_ctx.data.current_feeder_radps) < JAM_SPEED_THRESHOLD)
    {
        if (_ctx.data.jam_start_tick == 0)
            _ctx.data.jam_start_tick = now;
        else if (now - _ctx.data.jam_start_tick >= pdMS_TO_TICKS(JAM_DETECT_TIME_MS))
        {
            // 卡住确认 → 记录零点 → 设置初始化完成标志
            _ctx.data.trigger_offset = _ctx.data.current_feeder_rad;
            _ctx.data.is_initialized = true;
            _ctx.data.init_just_completed = true;  // 设置标志供应用层查询

            // 进入就绪状态
            _trig_goto(test_robot_data_ctx_t::trig_state_e::READY);
            return;
        }
    }
    else
    {
        _ctx.data.jam_start_tick = 0;
    }
}

// =========================================================
// 拨弹盘子状态: READY 进入
// =========================================================
void test_robot_t::_trig_ready_enter()
{
    _ctx.pid.feeder_pos_pid->clear();
    _ctx.pid.feeder_spd_pid->clear();
    // 位置环锁住当前位置
    _ctx.data.target_feeder_rad   = _ctx.data.current_feeder_rad;
    _ctx.data.target_feeder_radps = 0;
}

// =========================================================
// 拨弹盘子状态: READY 执行
//   等待命令 → 路由到 SINGLE / CONTINUE
// =========================================================
void test_robot_t::_trig_ready_execute()
{
    // 缓存命令 (本帧消费)
    bool trigger  = _ctx.cmd->feeder_trigger && !_ctx.data.cmd_feeder_trigger;
    bool fire     = _ctx.cmd->fire_enable;

    _ctx.data.cmd_feeder_trigger = _ctx.cmd->feeder_trigger;
    _ctx.data.cmd_fire_enable    = _ctx.cmd->fire_enable;

    // 不要每帧更新目标位置！目标位置在 enter 时已经设置好
    // 保持位置环锁定在进入时的位置
    // _ctx.data.target_feeder_rad 已经在 enter 中设置，这里不动
    _ctx.data.target_feeder_radps = 0;

    // --- 连发 ---
    if (fire)
    {
        _trig_goto(test_robot_data_ctx_t::trig_state_e::CONTINUE);
        return;
    }

    // --- 单发触发 ---
    if (trigger)
    {
        _trig_goto(test_robot_data_ctx_t::trig_state_e::SINGLE);
    }
}

// =========================================================
// 拨弹盘子状态: SINGLE 执行
//   位置环推进 SINGLE_SHOT_ANGLE, 检测卡弹, 到达目标时回 READY
// =========================================================
void test_robot_t::_trig_single_execute()
{
    float err = _ctx.data.target_feeder_rad - _ctx.data.current_feeder_rad;
    uint32_t now = xTaskGetTickCount();

    // (位置环由 _trig_fsm_execute 外层统一调用, 此处仅做状态判定)

    // --- 卡弹检测: 误差大 + 实际速度极低且持续 ---
    // 但如果已经恢复过2次，就不再检测卡弹，直接放弃
    if (_ctx.data.jam_recovery_count < 2 &&
        std::abs(err) > SINGLE_SHOT_ANGLE / 8.0f &&
        std::abs(_ctx.data.current_feeder_radps) < JAM_SPEED_THRESHOLD)
    {
        if (_ctx.data.jam_start_tick == 0)
            _ctx.data.jam_start_tick = now;
        else if (now - _ctx.data.jam_start_tick >= pdMS_TO_TICKS(JAM_DETECT_TIME_MS))
        {
            // 卡弹 → 进入恢复状态
            _ctx.data.jam_recovery_count++;
            _ctx.data.state_before_jam = test_robot_data_ctx_t::trig_state_e::SINGLE;
            _trig_goto(test_robot_data_ctx_t::trig_state_e::JAM_RECOVERY);
            return;
        }
    }
    else
    {
        _ctx.data.jam_start_tick = 0;
    }

    // --- 到位判定: 角度误差 < 阈值 或 超时 或 速度很低 ---
    bool angle_reached = std::abs(err) < SINGLE_DONE_ANGLE_THRESHOLD;
    bool timeout = (now - _ctx.data.single_start_tick) > pdMS_TO_TICKS(SINGLE_DONE_TIMEOUT_MS);
    // 如果误差不太大且速度很低，也认为到位（可能是卡住了或者力矩不够）
    bool stalled = (std::abs(err) < SINGLE_SHOT_ANGLE / 4.0f) &&
                   (std::abs(_ctx.data.current_feeder_radps) < 0.5f) &&
                   ((now - _ctx.data.single_start_tick) > pdMS_TO_TICKS(200));

    if (angle_reached || timeout || stalled)
    {
        _ctx.data.fire_count++;
        _ctx.data.last_shot_feeder_rad = _ctx.data.current_feeder_rad;
        _trig_goto(test_robot_data_ctx_t::trig_state_e::DONE);
    }
}

// =========================================================
// 拨弹盘子状态: CONTINUE 执行
//   速度环恒转速, 检测卡弹
// =========================================================
void test_robot_t::_trig_continue_execute()
{
    uint32_t now = xTaskGetTickCount();

    // 退出连发 → DONE
    if (!_ctx.cmd->fire_enable)
    {
        // 对齐到前方最近的槽位
        float relative = _ctx.data.current_feeder_rad - _ctx.data.trigger_offset;
        int32_t count  = static_cast<int32_t>(relative / SINGLE_SHOT_ANGLE) + 1;
        _ctx.data.target_feeder_rad =
            _ctx.data.trigger_offset + count * SINGLE_SHOT_ANGLE;
        _trig_goto(test_robot_data_ctx_t::trig_state_e::DONE);
        return;
    }

    // 速度环
    _ctx.data.target_feeder_radps = _ctx.cmd->feeder_speed;

    // --- 物理发弹计数: 角度跨过 SINGLE_SHOT_ANGLE 边界 → fire_count++ ---
    {
        float d = _ctx.data.current_feeder_rad - _ctx.data.last_shot_feeder_rad;
        while (d >= SINGLE_SHOT_ANGLE)
        {
            _ctx.data.fire_count++;
            _ctx.data.last_shot_feeder_rad += SINGLE_SHOT_ANGLE;
            d -= SINGLE_SHOT_ANGLE;
        }
    }

    // --- 卡弹检测: 速度过低 ---
    if (std::abs(_ctx.data.current_feeder_radps) < JAM_SPEED_THRESHOLD)
    {
        if (_ctx.data.jam_start_tick == 0)
            _ctx.data.jam_start_tick = now;
        else if (now - _ctx.data.jam_start_tick >= pdMS_TO_TICKS(JAM_DETECT_TIME_MS))
        {
            // 卡弹 → 进入恢复状态
            _ctx.data.state_before_jam = test_robot_data_ctx_t::trig_state_e::CONTINUE;
            _trig_goto(test_robot_data_ctx_t::trig_state_e::JAM_RECOVERY);
            return;
        }
    }
    else
    {
        _ctx.data.jam_start_tick = 0;
    }
}

// =========================================================
// 拨弹盘子状态: JAM_RECOVERY 执行
//   无力一段时间后重新尝试
// =========================================================
void test_robot_t::_trig_jam_recovery_execute()
{
    uint32_t now = xTaskGetTickCount();

    // 无力状态 (零力矩)
    _ctx.data.out_feeder_torque = 0;

    // 恢复时间到 → 返回卡弹前的状态
    if (now - _ctx.data.jam_recovery_start_tick >= pdMS_TO_TICKS(JAM_RECOVERY_TIME_MS))
    {
        _trig_goto(_ctx.data.state_before_jam);
    }
}

// =========================================================
// 拨弹盘子状态: DONE 执行
//   直接回到 READY，由 READY enter 处理位置锁定
// =========================================================
void test_robot_t::_trig_done_execute()
{
    // 直接回到就绪，不在这里设置目标位置
    // _trig_ready_enter() 会清除PID并锁定当前位置
    _trig_goto(test_robot_data_ctx_t::trig_state_e::READY);
}

// =========================================================
// 拨弹盘子状态机调度
// =========================================================
void test_robot_t::_trig_fsm_execute()
{
    using s_t = test_robot_data_ctx_t::trig_state_e;

    // --- 优先检查: 左拨杆在UP时, 强制回到 IDLE 状态 ---
    if (_ctx.cmd->force_stop)
    {
        if (_ctx.data.trig_state != s_t::IDLE)
        {
            _trig_goto(s_t::IDLE);
            return;
        }
    }

    switch (_ctx.data.trig_state)
    {
    case s_t::IDLE:          _trig_idle_execute();          break;
    case s_t::INIT_FORWARD:  _trig_init_forward_execute();  break;
    case s_t::READY:         _trig_ready_execute();         break;
    case s_t::SINGLE:        _trig_single_execute();        break;
    case s_t::CONTINUE:      _trig_continue_execute();      break;
    case s_t::JAM_RECOVERY:  _trig_jam_recovery_execute();  break;
    case s_t::DONE:          _trig_done_execute();          break;
    }

    // JAM_RECOVERY 状态: 直接输出零力矩, 不执行PID
    if (_ctx.data.trig_state == s_t::JAM_RECOVERY)
    {
        _ctx.data.out_feeder_torque = 0;
        return;
    }

    // 统一位置环 → 速度环 → 扭矩
    // CONTINUE / INIT_FORWARD: 纯速度模式, target_feeder_radps 由子状态直接给定
    if (_ctx.data.trig_state != s_t::CONTINUE &&
        _ctx.data.trig_state != s_t::INIT_FORWARD)
    {
        _ctx.data.target_feeder_radps =
            _ctx.pid.feeder_pos_pid->calculate(_ctx.data.target_feeder_rad,
                                                _ctx.data.current_feeder_rad);
    }
    // 速度环 → 扭矩
    _ctx.data.out_feeder_torque =
        _ctx.pid.feeder_spd_pid->calculate(_ctx.data.target_feeder_radps,
                                            _ctx.data.current_feeder_radps);
}

// =========================================================
// 拨弹盘控制入口 (由 active state 调用)
// =========================================================
void test_robot_t::_feeder_control()
{
    // 消费命令并驱动子状态机
    _trig_fsm_execute();
}

// =========================================================
// 发送电机扭矩命令
// =========================================================
void test_robot_t::_send_motor_command() const
{
    // 云台
    _ctx.motor.pitch->send_torque(_ctx.data.out_pitch_torque);

    // 摩擦轮
    _ctx.motor.friction[0]->send_torque(_ctx.data.out_friction_torque[0]);
    _ctx.motor.friction[1]->send_torque(_ctx.data.out_friction_torque[1]);

    // 拨弹盘
    _ctx.motor.feeder->send_torque(_ctx.data.out_feeder_torque);
}

// =========================================================
// 状态机执行 (每1ms调用一次)
// =========================================================
void test_robot_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_passive);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_state_active);

    _main_fsm.execute(this);
}

} // namespace pyro
