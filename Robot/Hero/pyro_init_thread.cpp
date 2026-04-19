#include "pyro_bsp_uart.h"
#include "pyro_can_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_image_drv.h"
#include "pyro_ins.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"
#include "pyro_sr04_drv.h"
#include "pyro_vt03_rc_drv.h"

namespace pyro
{
extern "C"
{
    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;


    void pyro_init_thread(void *argument)
    {
        dwt_drv_t::init(480); // Initialize DWT at 480 MHz

        can1_drv = new can_drv_t(&hfdcan1);
        can2_drv = new can_drv_t(&hfdcan2);
        can3_drv = new can_drv_t(&hfdcan3);
        can1_drv->init();
        can2_drv->init();
        can3_drv->init();
        can1_drv->start(); // NOLINT
        can2_drv->start(); // NOLINT
        can3_drv->start(); // NOLINT

        ins_drv = ins_drv_t::get_instance();
        ins_drv->init();

#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
#endif

#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        supercap_drv_t::get_instance()->start_rx();
#endif

#ifdef AUTOAIM_UART
        AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
#endif

#ifdef IMAGE_UART
        // IMAGE_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
        //                  UART_PARITY_NONE);
        image_drv_t::get_instance().start();
        image_drv_t::get_instance().init();
#endif

#ifdef CUSTOM_UART
        CUSTOM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                          UART_PARITY_NONE);
#endif

#ifdef SR04_UART
        SR04_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        sr04_drv::get_instance().init();
        sr04_drv::get_instance().start();
#endif

        vTaskDelete(nullptr);
    }
}
} // namespace pyro