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

    while (1) {
        uint32_t wait       = 0x00fffff;
        rpm_values_t values = { 0 };
        db_rpm_get_values(&values);
        printf(
            "Left  - speed: %i, RPM: %i, RPS: %i\n",
            values.left.speed,
            values.left.rpm,
            values.left.rps
        );
        printf(
            "Right - speed: %i, RPM: %i, RPS: %i\n",
            values.right.speed,
            values.right.rpm,
            values.right.rps
        );
        while (wait--);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
