# =========================================================================
# 1. 显式定义当前编译的板子类型（发射测试为单板控制，无需区分GIMBAL/CHASSIS）
# =========================================================================
set(BOARD "CHASSIS_BOARD")

# =========================================================================
# 2. 声明 Launcher_test 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Launcher_test OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Launcher_test PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件 =====================
target_sources(Launcher_test
        PRIVATE
        # === Application ===
        Application/pyro_gimbal_launcher_app.cpp

        # === Gimbal 模块 (双轴云台 + 摩擦轮 + 拨弹盘) ===
        Gimbal/pyro_test_robot.cpp
        Gimbal/fsm/pyro_test_active_state.cpp
        Gimbal/fsm/pyro_test_passive_state.cpp

        # === 顶层入口 ===
        pyro_mission_planer.cpp
        pyro_init_thread.cpp
)

# 头文件包含路径
target_include_directories(Launcher_test
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/Application
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# DR16 遥控器串口定义
target_compile_definitions(PYRo PUBLIC
        DR16_UART=PYRO_UART5
)

# ===================== 链接依赖库 =====================
target_link_libraries(Launcher_test
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
