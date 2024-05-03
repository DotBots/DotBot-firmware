/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the SAADC api.
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include "saadc.h"
#include "timer_hf.h"

//=========================== main =============================================

int main(void) {
    db_timer_hf_init(0);
    db_saadc_init(DB_SAADC_RESOLUTION_12BIT);

    while (1) {
        uint16_t value;
        db_saadc_read(DB_SAADC_INPUT_AIN0, &value);
        printf("AIN0: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_AIN1, &value);
        printf("AIN1: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_AIN2, &value);
        printf("AIN2: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_AIN3, &value);
        printf("AIN3: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_AIN4, &value);
        printf("AIN4: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_VDDH, &value);
        printf("VDDH: %i\n", value);
        db_saadc_read(DB_SAADC_INPUT_VDD, &value);
        printf("VDD : %i\n", value);
        db_timer_hf_delay_ms(0, 100);
    }
}
