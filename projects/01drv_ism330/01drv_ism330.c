/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to use the ISM330 IMU available on the DotBot
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include "board.h"
#include "board_config.h"
#include "timer_hf.h"
#include "ism330.h"

//=========================== variables ========================================

static ism330_acc_data_t  acc_data;
static ism330_gyro_data_t gyro_data;

//=========================== main =============================================

int main(void) {

    db_board_init();
    // Init the timer
    db_timer_hf_init(0);
    // Init the  IMU
    db_ism330_init(&db_sda, &db_scl);

    // read accelerometer and gyroscope data in a loop
    while (1) {
        // Read Accelerometer data
        db_ism330_accel_read(&acc_data);
        db_timer_hf_delay_ms(0, 250);

        // Read Gyroscope data
        db_ism330_gyro_read(&gyro_data);
        db_timer_hf_delay_ms(0, 250);
    }
}
