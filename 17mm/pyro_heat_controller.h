#ifndef PYRO_HEAT_CONTROLLER_H
#define PYRO_HEAT_CONTROLLER_H

#include <algorithm>
#include <cstdint>

namespace pyro
{

class HeatController
{
  public:
    static constexpr float HEAT_PER_BULLET    = 10.0f;
    static constexpr float BULLETS_PER_CIRCLE = 8.0f;
    static constexpr float SAFE_MARGIN        = 150.0f;
    static constexpr uint32_t REFEREE_DELAY_MS = 200;

    HeatController() = default;

    void tickCooling(float dt)
    {
        if (_coolingRate > 0.0f && _localHeat > 0.0f)
        {
            _localHeat -= _coolingRate * dt;
            if (_localHeat < 0.0f)
                _localHeat = 0.0f;
        }
    }

    void syncWithReferee(uint16_t refHeat, uint16_t refLimit,
                         uint16_t refCoolingRate, uint32_t current_time_ms)
    {
        if (static_cast<float>(refLimit) <= 0.0f)
            return;

        _heatLimit   = static_cast<float>(refLimit);
        _coolingRate = static_cast<float>(refCoolingRate);

        if ((current_time_ms - _lastShotTimeMs) > REFEREE_DELAY_MS)
            _localHeat = static_cast<float>(refHeat);
        else
            _localHeat = std::max(_localHeat, static_cast<float>(refHeat));
    }

    void recordBulletShot(uint32_t current_time_ms)
    {
        _localHeat += HEAT_PER_BULLET;
        _lastShotTimeMs = current_time_ms;
    }

    [[nodiscard]] bool canShootSingle() const
    {
        if (_heatLimit <= 0.0f)
            return true;
        return (_localHeat + HEAT_PER_BULLET) <= (_heatLimit - HEAT_PER_BULLET * 2.0f);
    }

    [[nodiscard]] bool isApproachingHeatLimit() const
    {
        if (_heatLimit <= 0.0f)
            return false;
        return (_localHeat + HEAT_PER_BULLET * 2.0f) > (_heatLimit);
    }

    // 返回拨弹盘目标角速度 (rad/s)。三区间: 安全区=全速, 维持区=冷却速率受限, 临界区=制动
    [[nodiscard]] float getSafeBurstRadps(float maxRadps) const
    {
        // 1. 无热量限制，直接返回最大转速
        if (_heatLimit <= 0.0f)
            return maxRadps;

        // ==================== 肖特基触发器滞回阈值 ====================
        const float heat_upper = _heatLimit - HEAT_PER_BULLET * 2.0f;  // 过热阈值（切低速）
        const float heat_lower = _heatLimit - SAFE_MARGIN;            // 安全阈值（切高速）
        static float last_output = maxRadps; // 上一次的输出（用于滞回区保持）
        // ============================================================

        // 条件1：热量 ≥ 过热阈值 → 强制低速
        if (_localHeat + HEAT_PER_BULLET >= heat_upper)
        {
            last_output = 0;
            return 0;
        }
        // 条件2：热量 ≤ 安全阈值 → 恢复最大转速
        else if (_localHeat <= heat_lower)
        {
            last_output = maxRadps;
            return maxRadps;
        }
        // 条件3：中间滞回区 → 保持上一次的输出（这里默认保持低速更安全）
        else
        {
            return last_output;
        }
    }

    [[nodiscard]] float getLocalHeat() const { return _localHeat; }

  private:
    float _localHeat{0.0f};
    float _heatLimit{0.0f};
    float _coolingRate{0.0f};
    uint32_t _lastShotTimeMs{0};
};

} // namespace pyro

#endif // PYRO_HEAT_CONTROLLER_H
