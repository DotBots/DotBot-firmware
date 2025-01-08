/**
 * @file
 * @ingroup     samples_drv
 * @author      Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * @brief       This is an example on how to use the AS5048B driver.
 *
 * Float angles in degrees are printed periodically every 50ms
 *
 * @copyright   Inria, 2024
 *
 */
#include <stdio.h>
#include <nrf.h>

#include "timer_hf.h"
#include "as5048b.h"

//=========================== main =========================================

int main(void) {
    // Init the encoder I2C communication
    as5048b_init();
    // Init high frequency clock
    db_timer_hf_init(0);

    while (1) {
        float angle_degrees = as5048b_i2c_read_angle_degree();
        printf("Angle degrees: %.1f\n", angle_degrees);
        db_timer_hf_delay_ms(0, 50);
    }
}
