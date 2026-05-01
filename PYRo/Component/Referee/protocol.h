#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include <cstdint>
#include <cstddef>


namespace pyro
{

constexpr uint8_t HEADER_SOF    = 0xA5;
constexpr size_t FRAME_MAX_SIZE = 256;
constexpr size_t HEADER_SIZE    = 5; // sizeof(frame_header_t)
constexpr size_t CMD_SIZE       = 2;
constexpr size_t CRC16_SIZE     = 2;
constexpr size_t HEADER_CRC_LEN = HEADER_SIZE + CRC16_SIZE;
constexpr size_t HEADER_CRC_CMDID_LEN =
    HEADER_SIZE + CRC16_SIZE + sizeof(uint16_t);
constexpr size_t HEADER_CMDID_LEN = HEADER_SIZE + sizeof(uint16_t);

#pragma pack(push, 1)

// 命令码 (enum class)
enum class cmd_id : uint16_t
{
    // --- Rx: Server -> Robot ---
    GAME_STATE          = 0x0001, // 比赛状态数据
    GAME_RESULT         = 0x0002, // 比赛结果数据
    GAME_ROBOT_HP       = 0x0003, // 机器人血量数据
    FIELD_EVENTS        = 0x0101, // 场地事件数据
    REFEREE_WARNING     = 0x0104, // 裁判警告数据
    DART_INFO           = 0x0105, // 飞镖发射相关数据

    ROBOT_STATE         = 0x0201, // 机器人性能体系数据
    POWER_HEAT_DATA     = 0x0202, // 实时功率热量数据
    ROBOT_POS           = 0x0203, // 机器人位置
    BUFF_MUSK           = 0x0204, // 机器人增益
    AERIAL_ENERGY       = 0x0205, // 空中支援时间数据
    ROBOT_HURT          = 0x0206, // 伤害状态
    SHOOT_DATA          = 0x0207, // 实时射击信息
    BULLET_REMAINING    = 0x0208, // 子弹剩余发送数
    ROBOT_RFID          = 0x0209, // 机器人RFID状态
    DART_CLIENT_CMD     = 0x020A, // 飞镖选手端指令数据
    GROUND_ROBOT_POS    = 0x020B, // 地面机器人位置数据
    RADAR_MARK          = 0x020C, // 雷达标记进度数据
    SENTRY_INFO         = 0x020D, // 哨兵自主决策信息同步
    RADAR_INFO          = 0x020E, // 雷达自主决策信息同步

    // --- Tx/Rx: 交互 ---
    STUDENT_INTERACTIVE = 0x0301, // 机器人交互数据
    CUSTOM_CONTROLLER   = 0x0302, // [图传] 自定义控制器与机器人交互
    TINY_MAP_INTERACT   = 0x0303, // 选手端小地图交互数据
    MAP_RECEIVE_RADAR   = 0x0305, // 选手端小地图接收雷达数据
    CUSTOM_CLIENT_DATA  = 0x0306, // 自定义控制器与选手端交互数据
    MAP_RECEIVE_PATH    = 0x0307, // 选手端小地图接收路径数据
    MAP_RECEIVE_ROBOT   = 0x0308, // 选手端小地图接收机器人数据 (自定义UI消息)
};

// 交互子命令码范围 (enum class)
enum class interaction_sub_cmd : uint16_t
{
    // 客户端 UI 绘制 (Server -> Client)
    UI_CMD_DELETE    = 0x0100,
    UI_CMD_DRAW_1    = 0x0101,
    UI_CMD_DRAW_2    = 0x0102,
    UI_CMD_DRAW_5    = 0x0103,
    UI_CMD_DRAW_7    = 0x0104,
    UI_CMD_DRAW_CHAR = 0x0110,
    // 机器人间通信 (Robot -> Robot)
    ROBOT_COMM_START = 0x0200,
    ROBOT_COMM_END   = 0x02FF,
    // 自主决策指令
    SENTRY_CMD       = 0x0120, // [V1.2] 哨兵自主决策指令
    RADAR_CMD        = 0x0121, // [V1.2] 雷达自主决策指令
};

// 帧头结构
struct frame_header_t
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

/* ----------------- Structure Definitions ----------------- */

// 0x0001
struct game_status_t
{
    uint8_t game_type     : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

// 0x0002
struct game_result_t
{
    uint8_t winner;
};

// 0x0003
struct game_robot_hp_t
{
    uint16_t robot_1_hp;
    uint16_t robot_2_hp;
    uint16_t robot_3_hp;
    uint16_t robot_4_hp;
    uint16_t reserved_1;
    uint16_t robot_7_hp;
    uint16_t outpost_hp;
    uint16_t base_hp;
};

// [修复] 0x0101 场地事件数据
struct event_data_t
{
    uint32_t supply_zone                : 3; // bit 0-2: 己方补给区的占领状态
    uint32_t energy_mechanism_small     : 2; // bit 3-4: 己方小能量机关的激活状态
    uint32_t energy_mechanism_big       : 2; // bit 5-6: 己方大能量机关的激活状态
    uint32_t circular_high_ground       : 2; // bit 7-8: 己方中央高地的占领状态
    uint32_t trapezoidal_high_ground    : 2; // bit 9-10: 己方梯形高地的占领状态
    uint32_t dart_last_hit_time         : 9; // bit 11-19: 对方飞镖最后一次击中己方前哨站或基地的时间
    uint32_t dart_last_hit_target       : 3; // bit 20-22: 对方飞镖最后一次击中己方前哨站或基地的具体目标
    uint32_t center_buff_zone           : 2; // bit 23-24: 中心增益点的占领状态 (仅RMUL适用)
    uint32_t fortress_buff_zone         : 2; // bit 25-26: 己方堡垒增益点的占领状态
    uint32_t outpost_buff_zone          : 2; // bit 27-28: 己方前哨站增益点的占领状态
    uint32_t base_buff_zone             : 1; // bit 29: 己方基地增益点的占领状态
    uint32_t reserved                   : 2; // bit 30-31: 保留
};

// 0x0104
struct referee_warning_t
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};

// [修复] 0x0105 飞镖信息
struct dart_info_t
{
    uint8_t dart_remaining_time;
    uint16_t dart_last_hit_target  : 3;  // bit 0-2: 最近一次己方飞镖击中的目标
    uint16_t dart_target_hit_count : 3;  // bit 3-5: 对方最近被击中的目标累计被击中计次数
    uint16_t dart_aim_target       : 3;  // bit 6-8: 飞镖此时选定的击打目标
    uint16_t reserved              : 7;  // bit 9-15: 保留
};

// 0x0201 机器人状态数据 (增加 reserved 补齐字节对齐)
struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1; // bit 0: 云台电源输出情况
    uint8_t power_management_chassis_output : 1; // bit 1: 底盘电源输出情况
    uint8_t power_management_shooter_output : 1; // bit 2: 发射机构电源输出情况
    uint8_t reserved                        : 5; // bit 3-7: 保留，补齐1字节
};

// 0x0202
struct power_heat_data_t
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

// 0x0203
struct robot_pos_t
{
    float x;
    float y;
    float angle;
    uint32_t reserved;
};

// 0x0204
struct buff_info_t
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
};

// 0x0206
struct hurt_data_t
{
    uint8_t armor_id            : 4;
    uint8_t hp_deduction_reason : 4;
};

// 0x0207
struct shoot_data_t
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
    uint16_t launching_num;
};

// 0x0208
struct projectile_allowance_t
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

// [修复] 0x0209 机器人 RFID 状态 (补全32位与附带的状态2)
struct rfid_status_t
{
    uint32_t ally_base_buff                   : 1; // bit 0
    uint32_t ally_circular_high               : 1; // bit 1
    uint32_t enemy_circular_high              : 1; // bit 2
    uint32_t ally_trapezoidal_high            : 1; // bit 3
    uint32_t enemy_trapezoidal_high           : 1; // bit 4
    uint32_t ally_fly_ramp_front              : 1; // bit 5
    uint32_t ally_fly_ramp_back               : 1; // bit 6
    uint32_t enemy_fly_ramp_front             : 1; // bit 7
    uint32_t enemy_fly_ramp_back              : 1; // bit 8
    uint32_t ally_circular_high_down          : 1; // bit 9
    uint32_t ally_circular_high_up            : 1; // bit 10
    uint32_t enemy_circular_high_down         : 1; // bit 11
    uint32_t enemy_circular_high_up           : 1; // bit 12
    uint32_t ally_highway_down                : 1; // bit 13
    uint32_t ally_highway_up                  : 1; // bit 14
    uint32_t enemy_highway_down               : 1; // bit 15
    uint32_t enemy_highway_up                 : 1; // bit 16
    uint32_t ally_fortress_buff               : 1; // bit 17
    uint32_t ally_outpost_buff                : 1; // bit 18
    uint32_t ally_supply_no_overlap           : 1; // bit 19
    uint32_t ally_supply_overlap              : 1; // bit 20
    uint32_t ally_assembly_buff               : 1; // bit 21
    uint32_t enemy_assembly_buff              : 1; // bit 22
    uint32_t center_buff                      : 1; // bit 23
    uint32_t enemy_fortress_buff              : 1; // bit 24
    uint32_t enemy_outpost_buff               : 1; // bit 25
    uint32_t ally_tunnel_highway_down         : 1; // bit 26
    uint32_t ally_tunnel_highway_mid          : 1; // bit 27
    uint32_t ally_tunnel_highway_up           : 1; // bit 28
    uint32_t ally_tunnel_trapezoidal_low      : 1; // bit 29
    uint32_t ally_tunnel_trapezoidal_mid      : 1; // bit 30
    uint32_t ally_tunnel_trapezoidal_high     : 1; // bit 31

    uint8_t rfid_status_2;
};

// 0x020A
struct dart_client_cmd_t
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
};

// 0x020B
struct ground_robot_position_t
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
};

// 0x020C
struct radar_mark_data_t
{
    uint16_t mark_progress;
};

// 0x020D
struct sentry_info_t
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
};

// 0x020E
struct radar_info_t
{
    uint8_t radar_info;
};

// 0x0303
struct map_command_t
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
};

// -----------------------------------------------------------
// TX Structs
// -----------------------------------------------------------

// 0x0301 Header
struct interaction_header_t
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

// 0x0301 Payload Wrapper
struct robot_interaction_data_t
{
    interaction_header_t header;
    uint8_t user_data[112];
};

// 0x0308
struct custom_info_t
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};

// Main Data Holder
struct referee_data_t
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_hp_t game_robot_hp;
    event_data_t field_event;
    referee_warning_t referee_warning;
    dart_info_t dart_info;

    robot_status_t robot_status;
    power_heat_data_t power_heat;
    robot_pos_t robot_pos;
    buff_info_t buff;
    hurt_data_t hurt;
    shoot_data_t shoot;
    projectile_allowance_t allowance;
    rfid_status_t rfid;
    dart_client_cmd_t dart_client_cmd;
    ground_robot_position_t ground_robot_pos;
    radar_mark_data_t radar_mark;
    sentry_info_t sentry_info;
    radar_info_t radar_info;

    map_command_t map_command;
    robot_interaction_data_t robot_interaction;
};

#pragma pack(pop)

} // namespace pyro

#endif // ROBOMASTER_PROTOCOL_H