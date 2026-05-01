/**
 * @file pyro_board_drv.h
 * @brief 板间通信底层驱动 (支持位域内存映射与自动分包)
 */

#ifndef PYRO_BOARD_DRV_H
#define PYRO_BOARD_DRV_H

#include "pyro_can_drv.h"
#include "pyro_core_def.h"
#include "pyro_task.h"
#include <cstdint>

namespace pyro
{

class board_drv_t
{
  public:
    enum class role_t
    {
        GIMBAL,
        CHASSIS
    };

/* ================== 通信协议结构体定义 ================== */
#pragma pack(push, 1)

    /**
     * @brief [云台 -> 底盘] 控制指令数据包
     * @note 使用位域压榨空间，扩充只需在此往下添加变量
     */
    struct g2c_data_t
    {
        int8_t vx;
        int8_t vy;
        int8_t wz;

        // 将多个 bool 压缩进 1 个字节中 (位域定义)
        uint8_t active       : 1;
        uint8_t track_en     : 1;
        uint8_t leg_retract  : 1;
        uint8_t sling_mode   : 1; // 预留：吊射模式状态
        uint8_t reserved_bit : 4; // 凑满1字节，保持内存整齐

        // float target_pitch; // 扩充示例
    };

    /**
     * @brief [底盘 -> 云台] 反馈数据包
     */
    struct c2g_data_t
    {
        uint16_t chassis_power;
        uint8_t state_flags;
        // float capacitor_voltage; // 扩充示例
    };

#pragma pack(pop)
/* ======================================================= */

    // 定义基准 CAN ID (后续分包会自动累加 ID)
    static constexpr uint32_t G2C_BASE_ID = 0x101;
    static constexpr uint32_t C2G_BASE_ID = 0x105;

    // 编译器自动计算需要的 CAN 帧数量
    static constexpr uint8_t G2C_FRAME_CNT = (sizeof(g2c_data_t) + 7) / 8;
    static constexpr uint8_t C2G_FRAME_CNT = (sizeof(c2g_data_t) + 7) / 8;

    static board_drv_t &get_instance(role_t role = role_t::GIMBAL,
                                     can_hub_t::which_can can_ch = can_hub_t::can1);

    void start_rx() const;

    // 数据交互接口
    g2c_data_t &get_g2c_tx_data();
    c2g_data_t &get_c2g_tx_data();
    [[nodiscard]] const g2c_data_t &get_g2c_rx_data() const;
    [[nodiscard]] const c2g_data_t &get_c2g_rx_data() const;

    status_t send_data() const;
    [[nodiscard]] bool check_online() const;
    [[nodiscard]] role_t get_role() const
    {
        return _role;
    }

  private:
    explicit board_drv_t(role_t role, can_hub_t::which_can can_ch);
    ~board_drv_t();

    class board_task_t final : public task_base_t
    {
      public:
        explicit board_task_t(board_drv_t *owner_ptr)
            : task_base_t("board_drv_task", 128, 256, priority_t::NORMAL),
              _owner(owner_ptr)
        {
        }

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        board_drv_t *_owner;
    };

    role_t _role;
    can_hub_t::which_can _can_ch;
    board_task_t *_task;

    g2c_data_t _g2c_tx_payload{};
    c2g_data_t _c2g_tx_payload{};
    g2c_data_t _latest_g2c_rx{};
    c2g_data_t _latest_c2g_rx{};

    bool _is_online;
    float _last_rx_time_ms;

    void init_impl() const;
    void run_loop_impl();
};

} // namespace pyro

#endif // PYRO_BOARD_DRV_H