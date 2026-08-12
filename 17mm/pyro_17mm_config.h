#ifndef __PYRO_PYRO_17MM_CONFIG_H__
#define __PYRO_PYRO_17MM_CONFIG_H__

#ifndef TRIGGER_CONTINUE_HEAT_RECOVERY_CALI_EN
#define TRIGGER_CONTINUE_HEAT_RECOVERY_CALI_EN 0
#endif

namespace pyro
{

// constexpr float TRIGGER_UNJAM_RADPS      = 6.0f;   // 解堵速度
constexpr float TRIGGER_CONTINUOUS_RADPS = 10; // 连续发射速度（拨弹盘速度）

// 堵转判定
// constexpr float TRIGGER_BLOCK_RAD        = 0.2f; // 堵转判定弧度阈值
// constexpr float TRIGGER_BLOCK_RADPS      = 5.0f; // 堵转判定速度阈值
// constexpr uint16_t TRIGGER_BLOCK_TIME    = 300;  // 堵转判定时间阈值

constexpr float TRIGGER_GEAR_RATIO       = 36.0f; // M2006拨弹电机减速比

// 校准参数
constexpr float CALI_REVERSE_RADPS       = -3.0f;  // 校准反转速度 (rad/s)
constexpr float CALI_FORWARD_TARGET_RAD  = 0;   // 校准正转目标角度 (rad)
constexpr float CALI_BLOCK_THRESHOLD     = 0.5f;   // 堵转检测速度误差阈值 (50%)
constexpr uint16_t CALI_BLOCK_TIME_MS    = 1000;    // 堵转检测时间阈值 (ms)
constexpr float CALI_DONE_ANGLE_THRESHOLD = 0.01f;  // 校准完成角度误差阈值 (rad)

static constexpr float WHEEL_RADIUS = 0.03f;
static constexpr float TARGET_BULLET_SPEED = 18.5f;  // 目标弹速

inline float lin_v_to_radps(float v) { return v / WHEEL_RADIUS; }

extern pid_t *bullet_speed_pid;

}

#endif // __PYRO_PYRO_17MM_CONFIG_H__
