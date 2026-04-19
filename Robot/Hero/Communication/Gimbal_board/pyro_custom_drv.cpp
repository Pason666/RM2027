/**
 * @file pyro_custom_drv.cpp
 * @brief Implementation of Custom Client Communication Driver.
 */

#include "pyro_custom_drv.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_core_dma_heap.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h"
#include <cstring>

namespace pyro
{

status_t custom_drv_t::custom_task_t::init()
{
    if (_owner) { _owner->init_impl(); return PYRO_OK; }
    return PYRO_ERROR;
}

void custom_drv_t::custom_task_t::run_loop()
{
    if (_owner) { _owner->run_loop_impl(); }
}

#ifdef CUSTOM_UART
custom_drv_t &custom_drv_t::get_instance()
{
    static custom_drv_t instance(&CUSTOM_UART);
    return instance;
}
#endif

custom_drv_t::custom_drv_t(uart_drv_t *uart_handle)
    : _uart_drv(uart_handle), _task(nullptr), _tx_buffer(nullptr),
      _rx_msg_buf(nullptr), _is_online(false), _has_new_data(false),
      _first_frame(true), _last_seq(0)
{
    memset(&_latest_data, 0, sizeof(_latest_data));
    memset(&_tx_payload, 0, sizeof(_tx_payload));

    constexpr size_t buf_size = sizeof(tx_packet_t);
    _tx_buffer = static_cast<tx_packet_t *>(pvPortDmaMalloc(buf_size));

    if (_tx_buffer) memset(_tx_buffer, 0, buf_size);

    _task = new custom_task_t(this);
}

custom_drv_t::~custom_drv_t()
{
    if (_task) { _task->stop(); delete _task; _task = nullptr; }
    if (_uart_drv) _uart_drv->remove_rx_event_callback(reinterpret_cast<uint32_t>(this));
    if (_tx_buffer) { vPortFree(_tx_buffer); _tx_buffer = nullptr; }
    if (_rx_msg_buf) { vMessageBufferDelete(_rx_msg_buf); _rx_msg_buf = nullptr; }
}

void custom_drv_t::start_rx() const
{
    if (_task && _uart_drv && _tx_buffer) _task->start();
}

void custom_drv_t::init_impl()
{
    if (_rx_msg_buf == nullptr) _rx_msg_buf = xMessageBufferCreate(1024);
    if (_rx_msg_buf == nullptr) return;

    _uart_drv->add_rx_event_callback(
        [this](const uint8_t *p, uint16_t size, BaseType_t &task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
}

void custom_drv_t::run_loop_impl()
{
    while (true)
    {
        static rx_packet_t pkt;
        static size_t xReceivedBytes;

        if (xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), portMAX_DELAY) == sizeof(rx_packet_t))
        {
            _is_online = true;
            _last_rx_time_ms = dwt_drv_t::get_timeline_ms();
        }

        while (_is_online)
        {
            xReceivedBytes = xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt), pdMS_TO_TICKS(100));

            if (xReceivedBytes == sizeof(rx_packet_t))
            {
                if (error_check(&pkt) == PYRO_OK)
                {
                    unpack(&pkt);
                    float current_time = dwt_drv_t::get_timeline_ms();
                    if (_last_rx_time_ms > 0.0f) _comm_interval_ms = current_time - _last_rx_time_ms;
                    _last_rx_time_ms = current_time;
                }
            }
            else if (xReceivedBytes == 0)
            {
                _is_online = false;
                _first_frame = true; // 掉线后重置首次标志，以便重新上线时直接接收第一包
                _comm_interval_ms = 0.0f;
                _last_rx_time_ms = 0.0f;
            }
        }
    }
}

bool custom_drv_t::rx_callback(const uint8_t *p_data, uint16_t size, BaseType_t &xHigherPriorityTaskWoken) const
{
    // ================== 调试专用代码开始 ==================
    static float debug_rx_intervals_ms[20] = {0.0f};
    static uint8_t debug_rx_idx = 0;
    static uint32_t last_rx_ticks = dwt_drv_t::get_current_ticks();

    float interval_ms = dwt_drv_t::get_delta_t(&last_rx_ticks) * 1000.0f;
    debug_rx_intervals_ms[debug_rx_idx] = interval_ms;
    debug_rx_idx = (debug_rx_idx + 1) % 20;
    // ================== 调试专用代码结束 ==================

    if (size == sizeof(rx_packet_t) && p_data[0] == FRAME_SOF)
    {
        xMessageBufferSendFromISR(_rx_msg_buf, p_data, sizeof(rx_packet_t), &xHigherPriorityTaskWoken);
        return true;
    }
    return false;
}

status_t custom_drv_t::error_check(const rx_packet_t *buf)
{
    if (!verify_crc16_check_sum(reinterpret_cast<uint8_t const *>(buf), sizeof(rx_packet_t)))
        return PYRO_ERROR;
    return PYRO_OK;
}

void custom_drv_t::unpack(const rx_packet_t *buf)
{
    uint16_t current_seq = buf->payload.seq;

    auto seq_diff = static_cast<int16_t>(static_cast<uint16_t>(current_seq - _last_seq));
    static uint64_t ab = 0;
    static uint64_t ac = 0;

    if (seq_diff > 1)
    {
        ab++;
    }
    if (seq_diff < 0)
    {
        ac++;
    }


    if (_first_frame || seq_diff > 0)
    {
        _first_frame = false;
        _last_seq = current_seq;

        // 拷贝并设置新数据标志位
        memcpy(&_latest_data, &buf->payload, sizeof(rx_data_t));
        _has_new_data = true;
    }
}

custom_drv_t::tx_data_t &custom_drv_t::get_tx_data() { return _tx_payload; }

status_t custom_drv_t::send_data() const
{
    if (!_tx_buffer || !_uart_drv) return PYRO_ERROR;

    _tx_buffer->header.sof = FRAME_SOF;
    memcpy(&_tx_buffer->payload, &_tx_payload, sizeof(tx_data_t));
    _tx_buffer->tailer.end = '\n';

    append_crc16_check_sum(reinterpret_cast<uint8_t *>(_tx_buffer), sizeof(tx_packet_t) - 1);

    return _uart_drv->write(reinterpret_cast<uint8_t *>(_tx_buffer), sizeof(tx_packet_t));
}

const custom_drv_t::rx_data_t &custom_drv_t::get_rx_data() const { return _latest_data; }
bool custom_drv_t::check_online() const { return _is_online; }
float custom_drv_t::get_comm_interval() const { return _comm_interval_ms; }

} // namespace pyro