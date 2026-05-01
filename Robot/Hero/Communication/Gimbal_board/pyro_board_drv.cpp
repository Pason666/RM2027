/**
 * @file pyro_board_drv.cpp
 */

#include "pyro_board_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_dwt_drv.h"
#include <cstring>

namespace pyro
{

status_t board_drv_t::board_task_t::init()
{
    if (_owner) {
        _owner->init_impl();
        return PYRO_OK;
    }
    return PYRO_ERROR;
}

void board_drv_t::board_task_t::run_loop()
{
    if (_owner) _owner->run_loop_impl();
}

board_drv_t &board_drv_t::get_instance(role_t role, can_hub_t::which_can can_ch)
{
    static board_drv_t instance(role, can_ch);
    return instance;
}

board_drv_t::board_drv_t(role_t role, can_hub_t::which_can can_ch)
    : _role(role), _can_ch(can_ch), _task(nullptr), _is_online(false), _last_rx_time_ms(0.0f)
{
    _task = new board_task_t(this);
}

board_drv_t::~board_drv_t()
{
    if (_task) {
        _task->stop();
        delete _task;
    }
}

void board_drv_t::start_rx() const { if (_task) _task->start(); }

void board_drv_t::init_impl()
{
    // 自动连续订阅所需的 ID 数量
    if (_role == role_t::CHASSIS) {
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i) {
            can_rx_drv_t::subscribe(_can_ch, G2C_BASE_ID + i);
        }
    } else {
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i) {
            can_rx_drv_t::subscribe(_can_ch, C2G_BASE_ID + i);
        }
    }
}

void board_drv_t::run_loop_impl()
{
    while (true) {
        std::array<uint8_t, 8> raw_data{};
        float current_time = dwt_drv_t::get_timeline_ms();

        if (_role == role_t::CHASSIS) {
            uint8_t* ptr = reinterpret_cast<uint8_t*>(&_latest_g2c_rx);
            for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i) {
                if (can_rx_drv_t::get_data(_can_ch, G2C_BASE_ID + i, raw_data)) {
                    // 自动重组内存切片
                    uint8_t copy_len = sizeof(g2c_data_t) - (i * 8);
                    if (copy_len > 8) copy_len = 8;
                    memcpy(ptr + (i * 8), raw_data.data(), copy_len);

                    _is_online = true;
                    _last_rx_time_ms = current_time;
                }
            }
        } else {
            uint8_t* ptr = reinterpret_cast<uint8_t*>(&_latest_c2g_rx);
            for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i) {
                if (can_rx_drv_t::get_data(_can_ch, C2G_BASE_ID + i, raw_data)) {
                    uint8_t copy_len = sizeof(c2g_data_t) - (i * 8);
                    if (copy_len > 8) copy_len = 8;
                    memcpy(ptr + (i * 8), raw_data.data(), copy_len);

                    _is_online = true;
                    _last_rx_time_ms = current_time;
                }
            }
        }

        // 超时离线检测
        if (_is_online && (current_time - _last_rx_time_ms > 100.0f)) {
            _is_online = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

board_drv_t::g2c_data_t &board_drv_t::get_g2c_tx_data() { return _g2c_tx_payload; }
board_drv_t::c2g_data_t &board_drv_t::get_c2g_tx_data() { return _c2g_tx_payload; }
const board_drv_t::g2c_data_t &board_drv_t::get_g2c_rx_data() const { return _latest_g2c_rx; }
const board_drv_t::c2g_data_t &board_drv_t::get_c2g_rx_data() const { return _latest_c2g_rx; }
bool board_drv_t::check_online() const { return _is_online; }

status_t board_drv_t::send_data() const
{
    auto* can_obj = can_hub_t::get_instance()->hub_get_can_obj(_can_ch);
    if (!can_obj) return PYRO_ERROR;

    if (_role == role_t::GIMBAL) {
        uint8_t* ptr = reinterpret_cast<uint8_t*>(const_cast<g2c_data_t*>(&_g2c_tx_payload));
        for (uint8_t i = 0; i < G2C_FRAME_CNT; ++i) {
            uint32_t current_id = G2C_BASE_ID + i;
            uint8_t send_len = sizeof(g2c_data_t) - (i * 8);
            if (send_len > 8) send_len = 8;

            can_tx_drv_t::clear(current_id);
            // 这里位长直接传入 send_len * 8，底层可以完美拷贝
            can_tx_drv_t::add_data_raw(current_id, send_len * 8, ptr + (i * 8));
            can_tx_drv_t::send(current_id, can_obj);
        }
    } else {
        uint8_t* ptr = reinterpret_cast<uint8_t*>(const_cast<c2g_data_t*>(&_c2g_tx_payload));
        for (uint8_t i = 0; i < C2G_FRAME_CNT; ++i) {
            uint32_t current_id = C2G_BASE_ID + i;
            uint8_t send_len = sizeof(c2g_data_t) - (i * 8);
            if (send_len > 8) send_len = 8;

            can_tx_drv_t::clear(current_id);
            can_tx_drv_t::add_data_raw(current_id, send_len * 8, ptr + (i * 8));
            can_tx_drv_t::send(current_id, can_obj);
        }
    }
    return PYRO_OK;
}

} // namespace pyro