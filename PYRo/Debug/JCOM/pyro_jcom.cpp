#include "pyro_jcom.h"
#include "pyro_core_dma_heap.h"
#include "task.h"
#include "cstring"
#include "pyro_bsp_uart.h"
#include "../../../Robot/Hero/Chassis/pyro_hybrid_chassis.h"

extern pyro::hybrid_chassis_t *hybrid_chassis_ptr;

// 外部声明弹速数组
extern float bullet_speed_array[];
extern uint8_t bullet_speed_count;

namespace pyro
{
jcom_drv_t::jcom_drv_t(uint8_t max_length, uart_drv_t *uart)
{
    _data_pack = static_cast<float *>(pvPortDmaMalloc(4 * max_length));
    _length    = 0;
    _jcom_uart = uart;
}

jcom_drv_t::~jcom_drv_t()
{
    if (_data_pack)
    {
        delete[] _data_pack;
        _data_pack = nullptr;
    }
}

#ifdef JCOM_DEBUG_PORT
jcom_drv_t &jcom_drv_t::get_instance(uint8_t max_length)
{
    static jcom_drv_t instance(
        max_length, &JCOM_DEBUG_PORT);
    return instance;
}
#endif

void jcom_drv_t::init()
{
}

void jcom_drv_t::add_data(float *data)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = 1;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void jcom_drv_t::add_data(float *data, const uint8_t len)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = len;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void jcom_drv_t::remove_data(const float *data)
{
    for (auto it = _data_nodes.begin(); it != _data_nodes.end(); ++it)
    {
        if (it->data == data)
        {
            _length -= it->size;
            _data_nodes.erase(it);
            break;
        }
    }
}

void jcom_drv_t::update_data()
{
    static uint8_t frame_head[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t offset               = 0;
    _data_pack[offset++]           = *reinterpret_cast<float *>(frame_head);
    for (const auto &[data, size] : _data_nodes)
    {
        for (uint8_t i = 0; i < size; ++i)
        {
            _data_pack[offset++] = data[i];
        }
    }
}

void jcom_drv_t::send()
{
    _jcom_uart->write(reinterpret_cast<uint8_t *>(_data_pack),
                      (_length + 1) * 4);

    // 发送回车换行
    uint8_t crlf[] = {0x0D, 0x0A};
    _jcom_uart->write(crlf, 2);
}

void jcom_drv_t::thread()
{
    uint8_t last_count = 0;

    while (true)
    {
        // 检测是否有新的弹速数据
        if (bullet_speed_count > last_count)
        {
            // 有新数据，添加弹速到数据包
            float latest_speed = bullet_speed_array[bullet_speed_count - 1];
            add_data(&latest_speed);

            // 更新并发送
            update_data();
            send();

            // 清空数据节点，准备下一次发送
            _data_nodes.clear();
            _length = 0;

            last_count = bullet_speed_count;
        }

        vTaskDelay(3);
    }
}

} // namespace pyro

extern "C" void pyro_jcom_task(void *arg)
{
#ifdef JCOM_DEBUG_PORT
    pyro::jcom_drv_t &jcom = pyro::jcom_drv_t::get_instance(15);
    jcom.thread();
#endif
    vTaskDelete(nullptr);
}