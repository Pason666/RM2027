# =========================================================================
# 1. 显式定义当前编译的板子类型（靶车为单板控制，无需区分GIMBAL/CHASSIS）
# =========================================================================
set(BOARD "CHASSIS_BOARD")

# =========================================================================
# 2. 声明 Target_Robot 目标文件库（OBJECT库，仅编译目标文件不链接）
# =========================================================================
add_library(Target_Robot OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Target_Robot PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件 =====================
target_sources(Target_Robot
        PRIVATE
        # === Application ===
        Application/pyro_chassis_app.cpp

        # === Chassis 模块 (舵-全向混合底盘) ===
        Chassis/pyro_target_robot.cpp
        Chassis/fsm/pyro_target_active_state.cpp
        Chassis/fsm/pyro_target_passive_state.cpp

        # === 顶层入口 ===
        pyro_mission_planer.cpp
        pyro_init_thread.cpp
)

# 头文件包含路径
target_include_directories(Target_Robot
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Application
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# DR16 遥控器串口定义
target_compile_definitions(PYRo PUBLIC
        DR16_UART=PYRO_UART5
)

# ===================== 链接依赖库 =====================
target_link_libraries(Target_Robot
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
