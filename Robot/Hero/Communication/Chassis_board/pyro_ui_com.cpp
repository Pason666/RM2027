#include "pyro_module_base.h"
#include "pyro_referee.h"
#include "pyro_ui_drv.h"
#include "pyro_can_drv.h"
#include "pyro_com_canrx.h"
#include "pyro_ui_com.h"
using namespace pyro;
static auto ui_draw_static() -> void;
static void uirxbooster();
static bool flush_flag = false;
static pyro::referee_drv_t *referee_ptr                   = nullptr;
static pyro::ui_drv_t *ui_ptr                             = nullptr;
struct ui_ctx_t
{
    bool lob_shoot_mode = false;
    bool fric1_online = false;
    bool fric2_online = false;
    float yaw_angle = 0.0f;//deg
    float pitch_angle = 0.0f;//deg
    float target_spd = 0.0f;//mps
    float super_cap_energy = 0.0f;
    static constexpr float super_cap_energy_max = 100.0f;
}ui_ctx;
static void uirxbooster()
{

}

static void info_update_test()
{
    static int time_count = 0;
    static int fric_count = 0;
    static bool fric_mode = false;
    static bool lob_mode = false;
    static int lob_count = 0;
    static int cap_count = 0;
    cap_count+=2;
    fric_count++;
    lob_count++;
    time_count++;
    if (fric_count % 10 == 0)
    {
        fric_mode = !fric_mode;
    }
    if (lob_count % 10 == 0)
    {
        lob_mode = !lob_mode;
    }
    if (time_count >=360)
    {
        time_count = 0;
    }
    ui_ctx.lob_shoot_mode = lob_mode;
    ui_ctx.fric1_online = fric_mode;
    ui_ctx.fric2_online = fric_mode;
    ui_ctx.yaw_angle = float(time_count)/2;
    ui_ctx.pitch_angle = float(time_count)/1.5f;
    ui_ctx.target_spd = time_count;
    ui_ctx.super_cap_energy = cap_count % 100 + 2;
}
static void ui_draw_static()
{
    //摩擦轮，吊射基础圆
    ui_ptr
        ->draw_circle("FRC",ui_operate::ADD, 2, pyro::ui_color::PINK,3, 1600+200,730,50)
        .draw_circle("LOB",ui_operate::ADD, 2, pyro::ui_color::WHITE,3, 1600+200,500+30,50)
    ;
    //超级电容进度条
    ui_ptr
        ->draw_rect("SCP",ui_operate::ADD, 2, pyro::ui_color::ORANGE,3,(1920/2-350),1080-940-60,(1920/2+350),1080-(940-60)-60)//700*60
        .draw_line("PBR",ui_operate::ADD,3,ui_color::ORANGE,50,(1920/2-350),1080-910-60,(1920/2-350),1080-910-60);

    // 1. 绘制静态文本标签 (注意名字不能重复)
    ui_ptr->draw_string("YAW", pyro::ui_operate::ADD, 3, pyro::ui_color::GREEN,
                    20, 2, 1450+80+200-25, 500-60, "YAW");
    vTaskDelay(pdMS_TO_TICKS(35));
    ui_ptr->draw_string("PIH", pyro::ui_operate::ADD, 3, pyro::ui_color::GREEN,
                        20, 2, 1450+80+200-25, 440-60, "PITCH");
    vTaskDelay(pdMS_TO_TICKS(35));


    // 2. 为动态数值提前进行 ADD 占位，赋予初始值，方便后续直接 MODIFY
    ui_ptr
        ->draw_float("YDG", pyro::ui_operate::ADD, 4, pyro::ui_color::GREEN, 20,
                     2, 1680+200, 500-60, 0.0f)//yaw_deg
        .draw_float("PDG", pyro::ui_operate::ADD, 4, pyro::ui_color::GREEN, 20,
                    2, 1680+200, 440-60, 0.0f)//pitch_deg
        .draw_float("SPD",pyro::ui_operate::ADD, 4, pyro::ui_color::GREEN,20,
                    2,1580+200-20,650,0.0f);
    ui_ptr->flush(); // 拼包发送

    vTaskDelay(pdMS_TO_TICKS(35));
}
extern "C"
{
static void ui_update_dynamic();

static void hero_ui_thread(void *argument)
{
    // 1. 阻塞等待裁判系统链路连通，并确保获取到了真实机器人ID
    while (!referee_ptr->is_online() || referee_ptr->get_robot_id() == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 2. 初始清理操作，并绘制静态结构
    ui_ptr->clear_all();
    vTaskDelay(pdMS_TO_TICKS(200));
    ui_draw_static();
    vTaskDelay(pdMS_TO_TICKS(100));

    while (true)
    {
        uirxbooster();

        info_update_test();

        if (referee_ptr->is_online())
        {
            if (flush_flag)
            {
                ui_ptr->clear_all();
                vTaskDelay(pdMS_TO_TICKS(200));
                ui_draw_static();
                vTaskDelay(pdMS_TO_TICKS(100));
                flush_flag = false;
            }
            else
            {
                ui_update_dynamic();
            }
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
bool fric_status_X = false;
bool lob_mode_crossline = false;
void ui_update_dynamic()
{
    // 修改摩擦轮速度值,YAW,PITCH值 (MODIFY)
    ui_ptr
        ->draw_float("YDG", pyro::ui_operate::MODIFY, 4, pyro::ui_color::GREEN, 20,
                     2, 1680+200-40-25, 500-60, ui_ctx.yaw_angle)//yaw_deg
        .draw_float("PDG", pyro::ui_operate::MODIFY, 4, pyro::ui_color::GREEN, 20,
                    2, 1680+200-40-25, 440-60, ui_ctx.pitch_angle)//pitch_deg
        .draw_float("SPD",pyro::ui_operate::MODIFY, 4, pyro::ui_color::GREEN,20,
                    2,1580+230-20,650,ui_ctx.target_spd);
    // 更新摩擦轮状态
    if (ui_ctx.fric1_online && ui_ctx.fric2_online)
    {
        if (fric_status_X)
        {
            fric_status_X = false;
            ui_ptr->clear_layer(5);
            vTaskDelay(pdMS_TO_TICKS(35));
            ui_ptr->draw_circle("FRC",ui_operate::MODIFY, 2, pyro::ui_color::GREEN,3, 1600+200,730,50);
        }
    }
    else
    {
        if (!fric_status_X)
        {
            fric_status_X = true;
            ui_ptr
            ->draw_line("FL1",ui_operate::ADD,5,ui_color::MAGENTA,3,1600-50+200,(730+50),1600+50+200,(730-50))
            .draw_line("FL2",ui_operate::ADD,5,ui_color::MAGENTA,3,1600+50+200,(730+50),1600-50+200,(730-50))
            .draw_circle("FRC",ui_operate::MODIFY, 2, pyro::ui_color::PINK,3, 1600+200,730,50)
            ;
        }
    }
    // 更新吊射模式
    if (ui_ctx.lob_shoot_mode)
    {
        if (!lob_mode_crossline)
        {
            lob_mode_crossline = true;
            ui_ptr
              ->draw_line("LL1",ui_operate::ADD,6,ui_color::ORANGE,3,1600+200,500+75+30,1600+200,500-75+30)
              .draw_line("LL2",ui_operate::ADD,6,ui_color::ORANGE,3,1600-75+200,500+30,1600+75+200,500+30)
              .draw_circle("LOB",ui_operate::MODIFY, 2, pyro::ui_color::YELLOW,3, 1600+200,500+30,50);
        }
    }
    else
    {
        if (lob_mode_crossline)
        {
            lob_mode_crossline = false;
            ui_ptr->clear_layer(6);
            vTaskDelay(pdMS_TO_TICKS(35));
            ui_ptr->draw_circle("LOB",ui_operate::MODIFY, 2, pyro::ui_color::WHITE,3, 1600+200,500+30,50);

        }
    }
    // 更新超级电容
    if (ui_ctx.super_cap_energy>=ui_ctx.super_cap_energy_max)
    {
        ui_ptr->draw_line("PBR",ui_operate::MODIFY,3,ui_color::CYAN,50,
         (1920/2-350),1080-910-60,
         (1920/2-350)+700,1080-910-60);
    }
    else
    {
        ui_ptr->draw_line("PBR",ui_operate::MODIFY,3,ui_color::ORANGE,50,
        (1920/2-350),1080-910-60,
        (1920/2-350)+700*(ui_ctx.super_cap_energy/ui_ctx.super_cap_energy_max),1080-910-60);
    }
    ui_ptr->flush();
}
}