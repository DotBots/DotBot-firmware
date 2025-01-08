#ifndef __SAADC_H
#define __SAADC_H

/**
 * @defgroup    bsp_saadc   SAADC
 * @ingroup     bsp
 * @brief       Functions to read the SAADC peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>
#include <nrf.h>

typedef enum {
    DB_SAADC_RESOLUTION_8BIT  = SAADC_RESOLUTION_VAL_8bit,   ///< 8-bit resolution
    DB_SAADC_RESOLUTION_10BIT = SAADC_RESOLUTION_VAL_10bit,  ///< 10-bit resolution
    DB_SAADC_RESOLUTION_12BIT = SAADC_RESOLUTION_VAL_12bit,  ///< 10-bit resolution
} db_saadc_resolution_t;

typedef enum {
    DB_SAADC_INPUT_AIN0 = SAADC_CH_PSELP_PSELP_AnalogInput0,  ///< AIN0 input
    DB_SAADC_INPUT_AIN1 = SAADC_CH_PSELP_PSELP_AnalogInput1,  ///< AIN1 input
    DB_SAADC_INPUT_AIN2 = SAADC_CH_PSELP_PSELP_AnalogInput2,  ///< AIN2 input
    DB_SAADC_INPUT_AIN3 = SAADC_CH_PSELP_PSELP_AnalogInput3,  ///< AIN3 input
    DB_SAADC_INPUT_AIN4 = SAADC_CH_PSELP_PSELP_AnalogInput4,  ///< AIN4 input
    DB_SAADC_INPUT_AIN5 = SAADC_CH_PSELP_PSELP_AnalogInput5,  ///< AIN5 input
    DB_SAADC_INPUT_AIN6 = SAADC_CH_PSELP_PSELP_AnalogInput6,  ///< AIN6 input
    DB_SAADC_INPUT_AIN7 = SAADC_CH_PSELP_PSELP_AnalogInput7,  ///< AIN7 input
    DB_SAADC_INPUT_VDD  = SAADC_CH_PSELP_PSELP_VDD,           ///< VDD input
    DB_SAADC_INPUT_VDDH = SAADC_CH_PSELP_PSELP_VDDHDIV5,      ///< VDDH input divided by 5
} db_saadc_input_t;                                           ///< SAADC input line

/**
 * @brief Initialize the SAADC peripheral
 */
void db_saadc_init(db_saadc_resolution_t resolution);

/**
 * @brief Read the SAADC value on a given input line
 */
void db_saadc_read(db_saadc_input_t input, uint16_t *value);

#endif
