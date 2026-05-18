/**
 * @file pyro_ui_com.cpp
 * @brief RoboMaster UI Component Implementation & FreeRTOS Thread
 */

#include "pyro_ui_com.h"
#include "pyro_ui_drv.h"
#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_can_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_supercap_drv.h"


using namespace pyro;

// ==========================================
// 全局变量定义区
// ==========================================
static pyro::referee_drv_t *referee_ptr = nullptr;
static pyro::ui_drv_t *ui_ptr           = nullptr;

static pyro::ui_ctx_t ui_ctx; // 全局数据上下文
static pyro::ui_com hero_ui;  // UI 绘制控制器组件

// ==========================================
// 辅助与测试函数
// ==========================================
static void uirxbooster()
{
    // 预留的 RX booster 逻辑
}

static void info_update_test()
{

    ui_ctx.sling_flag =
        board_drv_t::get_instance().get_g2c_rx_data().sling_mode;
    ui_ctx.fric_en_flag = board_drv_t::get_instance().get_g2c_rx_data().fric_en;
    ui_ctx.fric_error_flag =
        board_drv_t::get_instance().get_g2c_rx_data().fric_err;
    ui_ctx.yaw_rad =
        hybrid_chassis_t::instance()->get_ctx().data.current_yaw_error;
    ui_ctx.pitch_rad =
        static_cast<float>(
            board_drv_t::get_instance().get_g2c_rx_data().pitch_rad) /
        32768.0f;
    ui_ctx.target_shoot_spd =
        static_cast<float>(
            board_drv_t::get_instance().get_g2c_rx_data().target_shoot_spd) /
        10.0f;
    ui_ctx.super_cap_voltage = static_cast<float>(
        hybrid_chassis_t::instance()->get_ctx().cap_feedback.vot_cap);
    ui_ctx.refresh_flag = board_drv_t::get_instance().get_g2c_rx_data().ui_refresh;
    ui_ctx.track_en_flag = board_drv_t::get_instance().get_g2c_rx_data().track_en;
    ui_ctx.position_x = referee_ptr->get_data().robot_pos.x;
    ui_ctx.position_y = referee_ptr->get_data().robot_pos.y;
    memcpy(ui_ctx.mecanum_online,hybrid_chassis_t::instance()->get_ctx().data.wheel_online,4);

}

// ==========================================
// UI 布局与外观配置 (隐藏在匿名空间中)
// ==========================================
namespace
{
struct cfg_relative_pos
{
    static constexpr uint16_t x = 100, y = 600, chassis_scale = 70;
    static constexpr uint8_t chassis_layer = 2, gimbal_layer = 3;
    static constexpr uint8_t gimbal_r = 20, cannon_length = 50, cannon_width = 15;
    static constexpr uint8_t cannon_offset_r = 18;
};
struct cfg_trail
{
    static constexpr uint16_t x = 120, y = 800, r = 40;
    static constexpr uint8_t layer = 2, additional_layer = 7;
};
struct cfg_fric
{
    static constexpr uint16_t x = 1800, y = 730, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 5;
};
struct cfg_lob
{
    static constexpr uint16_t x = 1800, y = 530, r = 50;
    static constexpr uint8_t layer = 2, cross_layer = 6;
};
struct cfg_text
{
    static constexpr uint16_t yaw_x = 1800, yaw_y = 440;
    static constexpr uint16_t pitch_x = 1800, pitch_y = 380;
    static constexpr uint16_t spd_x = 1790, spd_y = 650;
    static constexpr uint16_t pos_x = 200, pos_y = 800;
    static constexpr uint16_t label_offset = 100;
    static constexpr uint8_t layer = 3, val_layer = 4;
};
struct cfg_cap
{
    static constexpr uint16_t cx = 960,
                              cy = 110; // 居中 1920/2, 底部 1080-910-60
    static constexpr uint16_t hw = 350, hh = 30;
    static constexpr uint8_t layer = 2;
};
} // namespace

// ==========================================
// ui_com 类方法实现
// ==========================================
void ui_com::init(ui_drv_t *drv)
{
    _drv = drv;
}

void ui_com::update_ctx(const ui_ctx_t &new_ctx)
{
    _last_ctx = _ctx;
    _ctx      = new_ctx;
    if (refresh_check())
    {
        refresh();
    }
}

void ui_com::draw_static()
{
    if (!_drv)
        return;

    // 1. 绘制基础背景圆
    _drv->draw_circle("FRC", ui_operate::ADD, cfg_fric::layer, ui_color::PINK,
                      3, cfg_fric::x, cfg_fric::y, cfg_fric::r)
        .draw_circle("LOB", ui_operate::ADD, cfg_lob::layer, ui_color::WHITE, 3,
                     cfg_lob::x, cfg_lob::y, cfg_lob::r);

    // 2. 绘制超级电容外框和进度条初始点
    _drv->draw_rect("SCP", ui_operate::ADD, cfg_cap::layer, ui_color::ORANGE, 3,
                    cfg_cap::cx - cfg_cap::hw, cfg_cap::cy + cfg_cap::hh,
                    cfg_cap::cx + cfg_cap::hw, cfg_cap::cy - cfg_cap::hh)
        .draw_line("PBR", ui_operate::ADD, cfg_cap::layer + 1, ui_color::ORANGE,
                   50, cfg_cap::cx - cfg_cap::hw, cfg_cap::cy,
                   cfg_cap::cx - cfg_cap::hw, cfg_cap::cy);
    // 3. 绘制静态文本标签
    _drv->draw_string("YAW", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::yaw_x - cfg_text::label_offset,
                      cfg_text::yaw_y, "YAW");
    _drv->draw_string("PIH", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pitch_x - cfg_text::label_offset,
                      cfg_text::pitch_y, "PITCH");
    _drv->draw_string("POX", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pos_x,
                      cfg_text::pos_y, "X:");
    _drv->draw_string("POY", ui_operate::ADD, cfg_text::layer, ui_color::GREEN,
                      20, 2, cfg_text::pos_x+100,
                      cfg_text::pos_y, "Y:");

    // 4. 浮点数值的 ADD 占位
    _drv->draw_float("YDG", ui_operate::ADD, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     0.0f)
        .draw_float("PDG", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                    cfg_text::pitch_y, 0.0f)
        .draw_float("SPD", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::spd_x, cfg_text::spd_y,
                    0.0f)
        .draw_float("PSX", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN,20,2,cfg_text::pos_x+30,cfg_text::pos_y,
                    0.0f)
        .draw_float("PSY", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN,20,2,cfg_text::pos_x+130,cfg_text::pos_y,
                    0.0f);
    // 5. 交叉线的ADD占位
    _drv->draw_line("FL1", ui_operate::ADD, cfg_fric::cross_layer,
                           ui_color::MAGENTA, 3, 1,1,1,1)
        .draw_line("FL2", ui_operate::ADD, cfg_fric::cross_layer,
                           ui_color::MAGENTA, 3, 1,1,1,1)
        .draw_line("LL1", ui_operate::ADD, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,1,1,1,1)
        .draw_line("LL2", ui_operate::ADD, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3, 1,1,1,1);
    // 5. 绘制越障模式台阶图案(静态)
    _drv->draw_line("TR1",ui_operate::ADD, cfg_trail::layer,ui_color::WHITE,3,
        cfg_trail::x-70,cfg_trail::y-18,
        cfg_trail::x+45,cfg_trail::y-18)
        .draw_line("TR2",ui_operate::ADD,cfg_trail::layer,ui_color::WHITE,3,
        cfg_trail::x-45,cfg_trail::y-18,
        cfg_trail::x-45,cfg_trail::y+18)
        .draw_line("TR3",ui_operate::ADD,cfg_trail::layer,ui_color::WHITE,3,
        cfg_trail::x-45,cfg_trail::y+18,
        cfg_trail::x+45,cfg_trail::y+18);
    // 5. 越障模式图标占位
    _drv->draw_circle("TF1",ui_operate::ADD, cfg_trail::additional_layer,
                    ui_color::MAGENTA,4,cfg_trail::x,cfg_trail::y,cfg_trail::r)
        .draw_line("TF2",ui_operate::ADD, cfg_trail::additional_layer,ui_color::MAGENTA,4,
                cfg_trail::x-cfg_trail::r*sqrt__2,cfg_trail::y-cfg_trail::r*sqrt__2,
                cfg_trail::x+cfg_trail::r*sqrt__2,cfg_trail::y+cfg_trail::r*sqrt__2)
        .draw_line("TT1",ui_operate::ADD, cfg_trail::additional_layer,ui_color::GREEN,4,
                    cfg_trail::x,cfg_trail::y-cfg_trail::r-10,
                    cfg_trail::x,cfg_trail::y+cfg_trail::r+10)
        .draw_line("TT2",ui_operate::ADD,cfg_trail::additional_layer,ui_color::GREEN,4,
                    cfg_trail::x,cfg_trail::y+cfg_trail::r+10,
                    cfg_trail::x-15,cfg_trail::y+cfg_trail::r-10)
        .draw_line("TT3",ui_operate::ADD,cfg_trail::additional_layer,ui_color::GREEN,4,
                    cfg_trail::x,cfg_trail::y+cfg_trail::r+10,
                    cfg_trail::x+15,cfg_trail::y+cfg_trail::r-10);
    // 6. 绘制云台旋转状态(静态底盘,炮塔圆)：
    _drv->draw_rect("CS1",ui_operate::ADD, cfg_relative_pos::chassis_layer,ui_color::WHITE,5,
        cfg_relative_pos::x,cfg_relative_pos::y,
        cfg_relative_pos::x+cfg_relative_pos::chassis_scale,
        cfg_relative_pos::y+cfg_relative_pos::chassis_scale*sqrt_2)
        .draw_circle("GMA",ui_operate::ADD, cfg_relative_pos::gimbal_layer,ui_color::WHITE,5,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2,
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2,
            cfg_relative_pos::gimbal_r)
    // 7. 云台图标占位
        .draw_line("GL1",ui_operate::ADD, cfg_relative_pos::chassis_layer,ui_color::WHITE,cfg_relative_pos::cannon_width,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+cfg_relative_pos::cannon_offset_r*cos(PI),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+cfg_relative_pos::cannon_offset_r*sin(PI),
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length)*cos(PI),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length)*sin(PI))
        .draw_line("GL2",ui_operate::ADD, cfg_relative_pos::gimbal_layer,ui_color::ALLY,cfg_relative_pos::cannon_width-5,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+cfg_relative_pos::cannon_offset_r*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+cfg_relative_pos::cannon_offset_r*sin(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length-5)*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length-5)*sin(_ctx.yaw_rad+PI/2));
    _drv->flush();
}

void ui_com::draw_dynamic()
{
    if (!_drv)
        return;

    draw_yaw();
    draw_pitch();
    draw_spd();
    draw_pos();
    _drv->flush();
    draw_fric_state();
    draw_lob_state();
    draw_super_cap();
    draw_trail_state();
    draw_relative_pos();
    _drv->flush();
    _force_refresh_flag = false;
}

void ui_com::refresh()
{
    if (!_drv)
        return;
    _drv->clear_all();
    _force_refresh_flag = true;
    draw_static();
    draw_dynamic();
}

void ui_com::draw_yaw()
{
    _drv->draw_float("YDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                     _ctx.yaw_rad);
}

void ui_com::draw_pitch()
{
    _drv->draw_float("PDG", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                     cfg_text::pitch_y, _ctx.pitch_rad);
}

void ui_com::draw_spd()
{
    _drv->draw_float("SPD", ui_operate::MODIFY, cfg_text::val_layer,
                     ui_color::GREEN, 20, 2, cfg_text::spd_x, cfg_text::spd_y,
                     _ctx.target_shoot_spd);
}

void ui_com::draw_pos()
{
    _drv->draw_float("PSX", ui_operate::MODIFY, cfg_text::val_layer,
                    ui_color::GREEN,20,2,cfg_text::pos_x+30,cfg_text::pos_y,
                    _ctx.position_x)
        .draw_float("PSY", ui_operate::MODIFY, cfg_text::val_layer,
                    ui_color::GREEN,20,2,cfg_text::pos_x+130,cfg_text::pos_y,
                    _ctx.position_y);
}
void ui_com::draw_fric_state()
{
    bool curr_ready = _ctx.fric_en_flag;
    bool last_ready = _last_ctx.fric_en_flag;
    float cross_line_reduce_k = sqrt__2;
    bool cross_line = false;
    ui_color circle_color = ui_color::PINK;
    ui_color cross_line_color = ui_color::MAGENTA;
    if (curr_ready!=last_ready || _force_refresh_flag)
    {
        if (curr_ready)
        {
            circle_color = ui_color::GREEN;
            if (_ctx.fric_error_flag)
            {
                cross_line_color = ui_color::ORANGE;
                cross_line_reduce_k = 1;
                cross_line = true;
            }
        }
        else
        {
            cross_line = true;
        }
        _drv->draw_circle("FRC", ui_operate::MODIFY, cfg_fric::layer,
                        circle_color, 3, cfg_fric::x, cfg_fric::y,
                        cfg_fric::r)
            .draw_line("FL1", ui_operate::MODIFY, cfg_fric::cross_layer,
                        cross_line_color, 3,
                        (cfg_fric::x - cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::y + cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::x + cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::y - cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line)
            .draw_line("FL2", ui_operate::MODIFY, cfg_fric::cross_layer,
                        cross_line_color, 3,
                        (cfg_fric::x - cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::y - cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::x + cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line,
                        (cfg_fric::y + cfg_fric::r*cross_line_reduce_k)*cross_line+!cross_line);
    }

}

void ui_com::draw_lob_state()
{
    if (_ctx.sling_flag != _last_ctx.sling_flag || _force_refresh_flag)
    {
        ui_color circle_color = ui_color::WHITE;
        bool cross_line = _ctx.sling_flag;
        if (_ctx.sling_flag)
        {

            circle_color = ui_color::YELLOW;
        }
        _drv->draw_circle("LOB", ui_operate::MODIFY, cfg_lob::layer,
                              circle_color, 3, cfg_lob::x, cfg_lob::y,
                              cfg_lob::r)
            .draw_line("LL1", ui_operate::MODIFY, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,
                           cfg_lob::x,
                           (cfg_lob::y + cfg_lob::r + 25)*cross_line,
                           cfg_lob::x,
                           (cfg_lob::y - cfg_lob::r - 25)*cross_line)
            .draw_line("LL2", ui_operate::MODIFY, cfg_lob::cross_layer,
                           ui_color::ORANGE, 3,
                           (cfg_lob::x - cfg_lob::r - 25)*cross_line,
                           cfg_lob::y,
                           (cfg_lob::x + cfg_lob::r + 25)*cross_line,
                           cfg_lob::y);
    }
}

void ui_com::draw_super_cap()
{
    float ratio = _ctx.super_cap_voltage / ui_ctx_t::super_cap_voltage_max;
    if (ratio > 1.0f)
        ratio = 1.0f;
    if (ratio < 0.0f)
        ratio = 0.0f;

    uint16_t start_x = cfg_cap::cx - cfg_cap::hw;
    uint16_t end_x   = start_x + static_cast<uint16_t>(2 * cfg_cap::hw * ratio);
    ui_color cap_color = (ratio >= 1.0f) ? ui_color::CYAN : ui_color::ORANGE;

    _drv->draw_line("PBR", ui_operate::MODIFY, cfg_cap::layer + 1, cap_color,
                    50, start_x, cfg_cap::cy, end_x, cfg_cap::cy);
}

void ui_com::draw_trail_state()
{



    if (_ctx.track_en_flag!= _last_ctx.track_en_flag || _force_refresh_flag)
    {
        _drv->draw_line("TT1",ui_operate::MODIFY, cfg_trail::additional_layer,ui_color::GREEN,4,
            cfg_trail::x,cfg_trail::y-(cfg_trail::r+10)*_ctx.track_en_flag,
            cfg_trail::x,cfg_trail::y+(cfg_trail::r+10)*_ctx.track_en_flag)
            .draw_line("TT2",ui_operate::MODIFY,cfg_trail::additional_layer,ui_color::GREEN,4,
            cfg_trail::x,cfg_trail::y+cfg_trail::r+10*_ctx.track_en_flag,
            cfg_trail::x-15*_ctx.track_en_flag,cfg_trail::y+cfg_trail::r-10*_ctx.track_en_flag)
            .draw_line("TT3",ui_operate::MODIFY,cfg_trail::additional_layer,ui_color::GREEN,4,
            cfg_trail::x,cfg_trail::y+cfg_trail::r+10*_ctx.track_en_flag,
            cfg_trail::x+15*_ctx.track_en_flag,cfg_trail::y+cfg_trail::r-10*_ctx.track_en_flag)
            .draw_circle("TF1",ui_operate::MODIFY, cfg_trail::additional_layer,
        ui_color::MAGENTA,4,cfg_trail::x,cfg_trail::y,cfg_trail::r*!_ctx.track_en_flag)
            .draw_line("TF2",ui_operate::MODIFY, cfg_trail::additional_layer,ui_color::MAGENTA,4,
            cfg_trail::x-cfg_trail::r*sqrt__2*!_ctx.track_en_flag,
            cfg_trail::y-cfg_trail::r*sqrt__2*!_ctx.track_en_flag,
            cfg_trail::x+cfg_trail::r*sqrt__2*!_ctx.track_en_flag,
            cfg_trail::y+cfg_trail::r*sqrt__2*!_ctx.track_en_flag);
    }

}

void ui_com::draw_relative_pos()
{
    ui_color rect_color = ui_color::WHITE;
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.mecanum_online[i] == false)
        {
            rect_color = ui_color::MAGENTA;
            break;
        }
    }
    _drv->draw_line("GL1",ui_operate::MODIFY, cfg_relative_pos::chassis_layer,ui_color::WHITE,cfg_relative_pos::cannon_width,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+cfg_relative_pos::cannon_offset_r*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+cfg_relative_pos::cannon_offset_r*sin(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length)*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length)*sin(_ctx.yaw_rad+PI/2))
        .draw_line("GL2",ui_operate::MODIFY, cfg_relative_pos::gimbal_layer,ui_color::ALLY,cfg_relative_pos::cannon_width-5,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+cfg_relative_pos::cannon_offset_r*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+cfg_relative_pos::cannon_offset_r*sin(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale/2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length-5)*cos(_ctx.yaw_rad+PI/2),
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale/2*sqrt_2+(cfg_relative_pos::cannon_offset_r+cfg_relative_pos::cannon_length-5)*sin(_ctx.yaw_rad+PI/2))
        .draw_rect("CS1",ui_operate::MODIFY, cfg_relative_pos::chassis_layer,rect_color,5,
            cfg_relative_pos::x,cfg_relative_pos::y,
            cfg_relative_pos::x+cfg_relative_pos::chassis_scale,
            cfg_relative_pos::y+cfg_relative_pos::chassis_scale*sqrt_2);
}
bool ui_com::refresh_check()
{
    return _last_ctx.refresh_flag!=_ctx.refresh_flag;
}
// ==========================================
// 线程与初始化 (外部 C 接口保持不变)
// ==========================================
extern "C"
{

    static void hero_ui_thread(void *argument)
    {
        // 1. 阻塞等待裁判系统链路连通
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // 2. 初始化封装器，并请求全局重绘
        hero_ui.init(ui_ptr);
        hero_ui.refresh();

        // 3. 主循环
        while (true)
        {
            uirxbooster();
            info_update_test(); // 将最新数据写入全局变量 ui_ctx

            if (referee_ptr->is_online())
            {
                hero_ui.update_ctx(ui_ctx);
                hero_ui.draw_dynamic();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void hero_ui_init(void *argument)
    {
        pyro::can_rx_drv_t::subscribe(can_hub_t::can2, 0x110);
        referee_ptr = pyro::referee_drv_t::get_instance();
        ui_ptr      = new pyro::ui_drv_t(referee_ptr);

        xTaskCreate(hero_ui_thread, "hero_ui_thread", 512, nullptr,
                    configMAX_PRIORITIES - 3, nullptr);

        vTaskDelete(nullptr);
    }

} // extern "C"