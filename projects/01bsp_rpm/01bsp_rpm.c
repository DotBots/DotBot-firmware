/**
 * @file 01bsp_rpm.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the Wheel revolution counter in the DotBot board.
 *
 * Load this program on your board. The LEDs should start blinking blue as the wheels are being turned.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "rpm.h"


//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    puts("RPM application");
    db_board_init();
    db_rpm_init();
    db_rpm_encoder_timers_start();

    while (1) {
        uint32_t left_speed     = db_rpm_get_left_speed();
        uint32_t left_rpm       = db_rpm_get_left_rpm();
        uint32_t left_rps       = db_rpm_get_left_rps();
        uint32_t right_speed    = db_rpm_get_right_speed();
        uint32_t right_rpm      = db_rpm_get_right_rpm();
        uint32_t right_rps      = db_rpm_get_right_rps();
        uint32_t wait           = 0x00fffff;
        printf(
            "Left  - speed: %i, RPM: %i, RPS: %i\n",
            left_speed,
            left_rpm,
            left_rps
        );
        printf(
            "Right - speed: %i, RPM: %i, RPS: %i\n",
            right_speed,
            right_rpm,
            right_rps
        );
        while (wait--);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
