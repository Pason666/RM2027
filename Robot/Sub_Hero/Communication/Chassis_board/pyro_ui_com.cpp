#include "pyro_ui_com.h"

#include "pyro_board_drv.h"
#include "pyro_can_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_mec_chassis.h"
#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"
#include "pyro_ui_drv.h"

using namespace pyro;

static pyro::referee_drv_t *referee_ptr = nullptr;
static pyro::ui_drv_t *ui_ptr           = nullptr;

static pyro::ui_ctx_t ui_ctx;
static pyro::ui_com hero_ui;

static void uirxbooster()
{
}

static void info_update_test()
{
    const auto &rx_data = board_drv_t::get_instance().get_g2c_rx_data();
    auto &chassis_ctx   = mec_chassis_t::instance()->get_ctx();

    ui_ctx.sling_flag       = rx_data.sling_mode;
    ui_ctx.fric_en_flag     = rx_data.fric_en;
    ui_ctx.fric_error_flag  = rx_data.fric_err;
    ui_ctx.yaw_rad          = chassis_ctx.data.current_yaw_error;
    ui_ctx.pitch_rad        =
        static_cast<float>(rx_data.pitch_rad) / 32768.0f;
    ui_ctx.target_shoot_spd =
        static_cast<float>(rx_data.target_shoot_spd) / 10.0f;
    ui_ctx.super_cap_voltage =
        static_cast<float>(chassis_ctx.cap_feedback.vot_cap);
    ui_ctx.refresh_flag = rx_data.ui_refresh;
}

namespace
{
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
    static constexpr uint16_t yaw_x = 1815, yaw_y = 440;
    static constexpr uint16_t pitch_x = 1815, pitch_y = 380;
    static constexpr uint16_t spd_x = 1790, spd_y = 650;
    static constexpr uint16_t label_offset = 210;
    static constexpr uint8_t layer = 3, val_layer = 4;
};

struct cfg_cap
{
    static constexpr uint16_t cx = 960, cy = 110;
    static constexpr uint16_t hw = 350, hh = 30;
    static constexpr uint8_t layer = 2;
};
} // namespace

void ui_com::init(ui_drv_t *drv)
{
    _drv = drv;
}

void ui_com::update_ctx(const ui_ctx_t &new_ctx)
{
    _last_ctx = _ctx;
    _ctx      = new_ctx;
}

void ui_com::draw_static()
{
    if (!_drv)
    {
        return;
    }

    _drv->draw_circle("FRC", ui_operate::ADD, cfg_fric::layer, ui_color::PINK,
                      3, cfg_fric::x, cfg_fric::y, cfg_fric::r)
        .draw_circle("LOB", ui_operate::ADD, cfg_lob::layer, ui_color::WHITE,
                     3, cfg_lob::x, cfg_lob::y, cfg_lob::r)
        .draw_rect("SCP", ui_operate::ADD, cfg_cap::layer, ui_color::ORANGE,
                   3, cfg_cap::cx - cfg_cap::hw, cfg_cap::cy + cfg_cap::hh,
                   cfg_cap::cx + cfg_cap::hw, cfg_cap::cy - cfg_cap::hh)
        .draw_line("PBR", ui_operate::ADD, cfg_cap::layer + 1,
                   ui_color::ORANGE, 50, cfg_cap::cx - cfg_cap::hw,
                   cfg_cap::cy, cfg_cap::cx - cfg_cap::hw, cfg_cap::cy);

    _drv->draw_string("YAW", ui_operate::ADD, cfg_text::layer,
                      ui_color::GREEN, 20, 2,
                      cfg_text::yaw_x - cfg_text::label_offset,
                      cfg_text::yaw_y, "YAW");
    _drv->draw_string("PIH", ui_operate::ADD, cfg_text::layer,
                      ui_color::GREEN, 20, 2,
                      cfg_text::pitch_x - cfg_text::label_offset,
                      cfg_text::pitch_y, "PITCH");

    _drv->draw_float("YDG", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::yaw_x, cfg_text::yaw_y,
                    0.0f)
        .draw_float("PDG", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::pitch_x,
                    cfg_text::pitch_y, 0.0f)
        .draw_float("SPD", ui_operate::ADD, cfg_text::val_layer,
                    ui_color::GREEN, 20, 2, cfg_text::spd_x, cfg_text::spd_y,
                    0.0f)
        .draw_line("FL1", ui_operate::ADD, cfg_fric::cross_layer,
                   ui_color::MAGENTA, 3, 1, 1, 1, 1)
        .draw_line("FL2", ui_operate::ADD, cfg_fric::cross_layer,
                   ui_color::MAGENTA, 3, 1, 1, 1, 1)
        .draw_line("LL1", ui_operate::ADD, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, 1, 1, 1, 1)
        .draw_line("LL2", ui_operate::ADD, cfg_lob::cross_layer,
                   ui_color::ORANGE, 3, 1, 1, 1, 1);
    _drv->flush();
}

void ui_com::draw_dynamic()
{
    if (!_drv)
    {
        return;
    }

    draw_yaw();
    draw_pitch();
    draw_spd();
    draw_fric_state();
    draw_lob_state();
    draw_super_cap();
    _drv->flush();
    _force_refresh_flag = false;
}

void ui_com::refresh()
{
    if (!_drv)
    {
        return;
    }
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

void ui_com::draw_fric_state()
{
    const bool curr_ready = _ctx.fric_en_flag;
    const bool last_ready = _last_ctx.fric_en_flag;
    float cross_line_reduce_k = sqrt__2;
    bool cross_line = false;
    ui_color circle_color = ui_color::PINK;
    ui_color cross_line_color = ui_color::MAGENTA;

    if (curr_ready != last_ready || _force_refresh_flag)
    {
        if (curr_ready)
        {
            circle_color = ui_color::GREEN;
            if (_ctx.fric_error_flag)
            {
                cross_line_color = ui_color::ORANGE;
                cross_line_reduce_k = 1.0f;
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
                       (cfg_fric::x - cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::y + cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::x + cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::y - cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line)
            .draw_line("FL2", ui_operate::MODIFY, cfg_fric::cross_layer,
                       cross_line_color, 3,
                       (cfg_fric::x + cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::y + cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::x - cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line,
                       (cfg_fric::y - cfg_fric::r * cross_line_reduce_k) *
                               cross_line +
                           !cross_line);
    }
}

void ui_com::draw_lob_state()
{
    if (_ctx.sling_flag != _last_ctx.sling_flag || _force_refresh_flag)
    {
        ui_color circle_color = ui_color::WHITE;
        const bool cross_line = _ctx.sling_flag;
        if (_ctx.sling_flag)
        {
            circle_color = ui_color::YELLOW;
        }

        _drv->draw_circle("LOB", ui_operate::MODIFY, cfg_lob::layer,
                          circle_color, 3, cfg_lob::x, cfg_lob::y,
                          cfg_lob::r)
            .draw_line("LL1", ui_operate::MODIFY, cfg_lob::cross_layer,
                       ui_color::ORANGE, 3, cfg_lob::x,
                       (cfg_lob::y + cfg_lob::r + 25) * cross_line,
                       cfg_lob::x,
                       (cfg_lob::y - cfg_lob::r - 25) * cross_line)
            .draw_line("LL2", ui_operate::MODIFY, cfg_lob::cross_layer,
                       ui_color::ORANGE, 3,
                       (cfg_lob::x - cfg_lob::r - 25) * cross_line,
                       cfg_lob::y,
                       (cfg_lob::x + cfg_lob::r + 25) * cross_line,
                       cfg_lob::y);
    }
}

void ui_com::draw_super_cap()
{
    float ratio = _ctx.super_cap_voltage / ui_ctx_t::super_cap_voltage_max;
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }

    const uint16_t start_x = cfg_cap::cx - cfg_cap::hw;
    const uint16_t end_x =
        start_x + static_cast<uint16_t>(2 * cfg_cap::hw * ratio);
    const ui_color cap_color =
        (ratio >= 1.0f) ? ui_color::CYAN : ui_color::ORANGE;

    _drv->draw_line("PBR", ui_operate::MODIFY, cfg_cap::layer + 1, cap_color,
                    50, start_x, cfg_cap::cy, end_x, cfg_cap::cy);
}

bool ui_com::refresh_check()
{
    return _last_ctx.refresh_flag != _ctx.refresh_flag;
}

extern "C"
{
    static void hero_ui_thread(void *argument)
    {
        while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        hero_ui.init(ui_ptr);
        hero_ui.refresh();

        while (true)
        {
            uirxbooster();
            info_update_test();

            if (referee_ptr->is_online())
            {
                if (hero_ui.refresh_check())
                {
                    hero_ui.refresh();
                }
                else
                {
                    hero_ui.update_ctx(ui_ctx);
                    hero_ui.draw_dynamic();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
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
}
