# CAN 扩展帧测试指南

## 修改内容总结

已完成对 CAN 驱动的扩展帧支持修改：

### 1. 头文件修改 (`pyro_can_drv.h`)
- `can_msg_buffer_t` 增加 `id_type_t` 枚举（STANDARD_ID / EXTENDED_ID）
- 构造函数增加 `type` 参数，默认为标准帧
- 新增 `get_id_type()` 方法
- `can_drv_t` 的 `send_msg()` 增加 `type` 参数
- `can_drv_t` 的 `handle_rx_msg()` 增加 `type` 参数
- 使用复合键 `register_key_t = std::pair<uint32_t, id_type_t>` 区分标准帧和扩展帧

### 2. 实现文件修改 (`pyro_can_drv.cpp`)
- 初始化函数配置标准帧和扩展帧过滤器
- 全局过滤器设置为同时接受标准帧和扩展帧
- 发送函数根据 `type` 参数设置 `tx_header.IdType`
- 接收中断处理扩展帧，自动识别帧类型
- 注册表使用复合键避免 ID 冲突

## 上位机测试方案

### 测试工具准备

推荐使用以下任一工具：
1. **PCAN-View** (PEAK CAN 分析仪配套软件)
2. **CANoe** (Vector 工具链)
3. **ZLG CANPro** (周立功 CAN 分析仪)
4. **USB-CAN 分析仪** + 配套上位机软件

### 测试步骤

#### 步骤 1：硬件连接
```
上位机 CAN 分析仪
    ↓
CAN_H ─────→ 板卡 CAN1_H
CAN_L ─────→ 板卡 CAN1_L
GND   ─────→ 板卡 GND

注意：确保 CAN 总线有 120Ω 终端电阻
```

#### 步骤 2：配置 CAN 波特率
- 标准波特率：**1 Mbps** (常用于机器人)
- 或根据你的 `fdcan.c` 配置的波特率

#### 步骤 3：启用测试任务
在你的初始化代码中调用：
```cpp
// 在 pyro_init_thread() 或 large_launcher_init() 中添加
extern void start_can_extended_test();
start_can_extended_test();
```

#### 步骤 4：上位机测试项目

##### 测试 1：接收板卡发送的帧
板卡会每 500ms 发送：
- **标准帧**: ID `0x456`, 数据 `[counter(4字节)] [AA BB CC DD]`
- **扩展帧**: ID `0x18FF5678`, 数据 `[counter(4字节)] [AA BB CC DD]`

**验证点**：
- [ ] 上位机能看到标准帧 0x456
- [ ] 上位机能看到扩展帧 0x18FF5678
- [ ] counter 递增正常
- [ ] 数据尾部始终是 AA BB CC DD

##### 测试 2：向板卡发送标准帧
用上位机发送：
- **ID**: `0x123` (标准帧)
- **数据**: `11 22 33 44 55 66 77 88`
- **周期**: 100ms 或单次发送

**验证点**：
- [ ] 板卡能接收到数据（可通过调试器查看 `std_msg_buffer`）
- [ ] `_is_fresh` 标志被置位
- [ ] 数据内容正确

##### 测试 3：向板卡发送扩展帧
用上位机发送：
- **ID**: `0x18FF1234` (扩展帧)
- **数据**: `AA BB CC DD EE FF 00 11`
- **周期**: 100ms 或单次发送

**验证点**：
- [ ] 板卡能接收到数据（可通过调试器查看 `ext_msg_buffer`）
- [ ] `_is_fresh` 标志被置位
- [ ] 数据内容正确

##### 测试 4：混合收发
同时进行：
1. 上位机发送标准帧 0x123 (100ms 周期)
2. 上位机发送扩展帧 0x18FF1234 (100ms 周期)
3. 观察板卡发出的标准帧 0x456
4. 观察板卡发出的扩展帧 0x18FF5678

**验证点**：
- [ ] 四种帧都正常收发
- [ ] 无丢包
- [ ] 无 ID 冲突（标准 0x123 和扩展 0x18FF1234 独立）

##### 测试 5：边界条件测试
1. **标准帧最大 ID**: 发送 `0x7FF` 标准帧
2. **扩展帧最大 ID**: 发送 `0x1FFFFFFF` 扩展帧
3. **ID 重叠测试**: 
   - 发送标准帧 `0x123`
   - 发送扩展帧 `0x00000123`（数值相同但帧类型不同）
   - 验证两者被正确区分

### 测试配置示例（PCAN-View）

#### 发送标准帧配置
```
Message Type: Standard (11-bit)
CAN ID: 0x123
DLC: 8
Data: 11 22 33 44 55 66 77 88
Cycle Time: 100 ms
```

#### 发送扩展帧配置
```
Message Type: Extended (29-bit)
CAN ID: 0x18FF1234
DLC: 8
Data: AA BB CC DD EE FF 00 11
Cycle Time: 100 ms
```

### 调试建议

#### 使用 VOFA+ 或 JCOM 查看数据
如果你的系统配置了调试串口，可以在测试代码中取消注释 `printf` 输出：

```cpp
// 在 can_extended_test_task 中启用
printf("STD RX [0x123]: %02X %02X %02X %02X %02X %02X %02X %02X\n", ...);
printf("EXT RX [0x18FF1234]: %02X %02X %02X %02X %02X %02X %02X %02X\n", ...);
```

#### 使用调试器监控
在 Ozone 或 STM32CubeIDE 中设置观察点：
```
std_msg_buffer->_is_fresh
std_msg_buffer->_buffer
ext_msg_buffer->_is_fresh
ext_msg_buffer->_buffer
```

### 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 收不到扩展帧 | 过滤器未配置 | 检查 `init()` 中扩展帧过滤器配置 |
| 发送失败返回 PYRO_BUSY | TX FIFO 满 | 检查发送频率，降低发送速率 |
| 标准帧和扩展帧混淆 | 注册表键值冲突 | 确认使用 `register_key_t` 复合键 |
| 接收数据错位 | 中断嵌套问题 | 检查 `taskENTER_CRITICAL_FROM_ISR` 使用 |

### 兼容性说明

修改后的驱动**向后兼容**：
- 所有现有代码（如 DJI 电机驱动）无需修改
- `send_msg()` 和 `can_msg_buffer_t` 构造函数的 `type` 参数都有默认值 `STANDARD_ID`
- 原有标准帧功能完全保留

### API 使用示例

```cpp
// 接收标准帧（兼容旧代码）
auto *std_msg = new can_msg_buffer_t(0x201);
can1_drv->register_rx_msg(std_msg);

// 接收扩展帧（新功能）
auto *ext_msg = new can_msg_buffer_t(0x18FF1234, can_msg_buffer_t::EXTENDED_ID);
can1_drv->register_rx_msg(ext_msg);

// 发送标准帧（兼容旧代码）
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
can1_drv->send_msg(0x200, data);

// 发送扩展帧（新功能）
can1_drv->send_msg(0x18FF5678, data, can_msg_buffer_t::EXTENDED_ID);
```

## 编译和烧录

1. 重新编译项目
2. 烧录到板卡
3. 启动 CAN 分析仪
4. 按照上述测试步骤验证

## 测试检查清单

- [ ] 标准帧发送正常
- [ ] 标准帧接收正常
- [ ] 扩展帧发送正常
- [ ] 扩展帧接收正常
- [ ] 标准帧和扩展帧可同时工作
- [ ] 相同数值不同类型的 ID 被正确区分
- [ ] 原有 DJI 电机等设备功能正常
