/**
 * @file
 * @ingroup     samples_drv
 * @author      Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * @brief       This is an example on how to use the AS5048B driver.
 *
 * Raw encoder 14-bit uint16_t values are continuously printed as debug output.
 *
 * @copyright   Inria, 2024
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "timer_hf.h"
#include "as5048b.h"

//=========================== main =========================================
/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Init the encoder I2C communication
    as5048b_init();
    // Init high frequency clock
    db_timer_hf_init();

    float angle_degrees;

    while (1) {
        as5048b_i2c_read_angle_degree(&angle_degrees);

        printf("Angle degrees: %.1f\n", angle_degrees);
        db_timer_hf_delay_ms(200);
    }
    // One last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
