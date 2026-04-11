#ifndef CAN_DRV_H
#define CAN_DRV_H

#include "fdcan.h"
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h" // 引入任务调度头文件以使用临界区宏
#include <array>
#include <cmsis_os.h>

#include "map.h"

namespace pyro
{
class can_msg_buffer_t
{
public:
    explicit can_msg_buffer_t(uint32_t id);
    ~can_msg_buffer_t();

    [[nodiscard]] uint32_t get_id() const;
    [[nodiscard]] bool is_fresh() const;
    void mark_read();
    void update_data(const uint8_t *data);
    [[nodiscard]] bool get_data(std::array<uint8_t, 8> &data) const;
    [[nodiscard]] TickType_t get_last_update_time() const;

private:
    uint32_t _id;
    std::array<uint8_t, 8> _buffer;
    volatile bool _is_fresh;
    volatile TickType_t _last_update_time;
};

class can_drv_t
{
    const uint8_t MAX_ID_REGIST_NUM = 32;
    using can_id_regist_t           = uint16_t;

public:
    explicit can_drv_t(FDCAN_HandleTypeDef *hfdcan);
    ~can_drv_t();

    status_t init();
    status_t start() const;
    status_t send_msg(uint32_t id, const uint8_t *data) const;
    status_t register_rx_msg(can_msg_buffer_t *msg_buffer);
    status_t handle_rx_msg(uint32_t id, const uint8_t *data);

private:
    FDCAN_HandleTypeDef *_hfdcan;
    map_t<uint32_t, can_msg_buffer_t *> _registerlist;
};

class can_hub_t
{
public:
    enum which_can
    {
        can1,
        can2,
        can3
    };

    static can_hub_t *get_instance();

    status_t hub_register_can_obj(FDCAN_HandleTypeDef *hfdcan,
                                  can_drv_t *can_drv);
    status_t hub_unregister_can_obj(FDCAN_HandleTypeDef *hfdcan);
    can_drv_t *hub_get_can_obj(which_can which_can);
    status_t hub_handle_callback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t identifier, const uint8_t *data);

private:
    can_hub_t();
    can_hub_t(const can_hub_t &)            = delete;
    can_hub_t &operator=(const can_hub_t &) = delete;

    map_t<FDCAN_HandleTypeDef *, can_drv_t *> _can_drv_map;
};
}; // namespace pyro

#endif