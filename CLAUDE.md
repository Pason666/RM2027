# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This is an STM32H723 embedded firmware project using CMake and ARM GCC toolchain.

**Build requires ROBOT_ID to be set:**
```bash
# Build for Hero robot (ID=1)
cmake -B build -DROBOT_ID=1
cmake --build build

# Other robot IDs:
# HERO=1, ENGINEER=2, INFANTRY1=3, INFANTRY2=4, SENTRY=5, UAV=6, DARTS=7, RADAR=8
# SUB_HERO=10, SUB_ENGINEER=20, SUB_INFANTRY=30, SUB_SENTRY=50
```

**Build options (CMake cache variables):**
- `DEMO_MODE=1` - Enable demo mode
- `DEBUG_MODE=1` - Enable debug mode
- `IMU_CALIBRATION_EN=1` - Enable IMU calibration mode

**Flash the firmware:**
Use an appropriate STM32 programmer (ST-Link, J-Link) with the generated `.elf`/`.bin` file.

## Architecture Overview

This is a RoboMaster competition robot firmware built on FreeRTOS with a layered architecture:

### Directory Structure
- `CubeMX/` - STM32CubeMX generated HAL code and FreeRTOS
- `PYRo/` - Framework layer (reusable across robots)
- `Robot/Hero/` - Hero-specific application logic

### PYRo Framework (`PYRo/`)

**Core modules (`PYRo/Core/`):**
- `FSM` - Template-based hierarchical state machine with Enter/Execute/Exit lifecycle and request-based state switching
- `Task` - C++ wrapper for FreeRTOS tasks with two-stage initialization (init stack separate from loop stack for memory efficiency)
- `Lock` - RAII mutex and read-write locks (write-priority strategy)
- `Memory` - DMA-safe heap allocator (`pvPortDmaMalloc`) for STM32H7 where DTCM is DMA-inaccessible
- `Config` - Global configuration macros (UART instances via `PYRO_UARTx`)
- `DataBoard` - Dynamic inter-task data sharing with read/write locks

**Peripheral drivers (`PYRo/Peripheral/`):**
- `CAN` - FDCAN driver with callback registration and ISR-to-instance mapping
- `UART` - DMA dual-buffer UART driver with callback system using `std::function` and owner-ID for removal
- `DWT` - High-resolution timer using ARM Cortex-M DWT cycle counter

**Component drivers (`PYRo/Component/`):**
- `Motor` - DJI motor (M3508/M2006) and DM motor drivers
- `RC` - Remote controller framework with priority-based preemption (VT03 > DR16), virtual controller mapping, button/switch state machines, and broker-based event subscription via `xTaskNotify`
- `INS` - IMU/attitude estimation driver
- `Referee` - RoboMaster referee system communication
- `Supercap` - Supercapacitor management

### Robot/Hero Application

**Subsystems:**
- `Booster` - Shooting mechanism (quad booster for Hero)
- `Gimbal` - Camera/aiming control (screw gimbal)
- `Chassis` - Movement (hybrid chassis)
- `Application/Gimbal_board/` - Gimbal board specific tasks
- `Application/Chassis_board/` - Chassis board specific tasks

**Initialization flow:**
- `pyro_init_thread.cpp` creates and starts all peripheral drivers (CAN1/2/3, INS, RC, Referee, Supercap)

## Key Patterns

**RC Event Subscription:**
```cpp
// Subscribe to button press (PRESS_DOWN for zero-latency, SINGLE_CLICK has confirmation delay)
pyro::btn_broker::subscribe(&vrc.keys.q, pyro::btn_event_t::PRESS_DOWN, task_handle, EVENT_BIT);
// In task loop: xTaskNotifyWait to receive events
```

**UART Callback Registration:**
```cpp
PYRO_UART1.add_rx_event_callback(
    [this](uint8_t *buf, uint16_t len, BaseType_t &woken) -> bool {
        return this->parse(buf, len, woken);
    },
    reinterpret_cast<uint32_t>(this)  // owner ID for removal
);
```

**FSM Usage:**
```cpp
// Define Context struct, inherit from state_t<Context>
// FSM handles Enter->Execute->Exit lifecycle and state switching via request_switch()
```

**DMA Memory:**
Use `pvPortDmaMalloc()` instead of `pvPortMalloc()` for any buffer passed to DMA (placed in SRAM1/2/3, not DTCM).

**Task Definition:**
```cpp
class MyTask : public pyro::task_base_t {
public:
    MyTask() : task_base_t("name", init_stack, loop_stack, priority_t::NORMAL) {}
protected:
    void init() override { /* runs once in temp task */ }
    void run_loop() override { while(1) { /* main loop */ } }
};
```

## Code Style

- C++17 with LLVM-based clang-format (Allman braces, 80 column limit, right-aligned pointers)
- Chinese comments alongside English documentation in README files
- Allman brace style with 4-space indentation