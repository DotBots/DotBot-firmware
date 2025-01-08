#ifndef __MOTORS_H
#define __MOTORS_H

/**
 * @defgroup    drv_motors  Motors driver
 * @ingroup     drv
 * @brief       Control the DC motors
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <nrf.h>

/**
 * @brief Configures the PMW0 peripheral to work with the onboard DotBot RGB Motor driver
 *
 * The DotBot uses a DRV8833 dual H-bridge driver with a 4 pmw control interface.
 * the PWM0 peripheral is used to generate the pwm signals it requires.
 *
 * PWM frequency = 10Khz
 * PWM resolution = 100 units (1us resolution)
 *
 */
void db_motors_init(void);

/**
 * @brief Set the percentage speed of the right and left motors on the DotBot
 *
 *  Each motor input variable receives a percentage speed from -100 to 100.
 *  Positive values turn the motor forward.
 *  Negative values turn the motor backward.
 *  Zero, stops the motor
 *
 * @param[in] l_speed speed of the left motor [-100, 100]
 * @param[in] r_speed speed of the left motor [-100, 100]
 */
void db_motors_set_speed(int16_t l_speed, int16_t r_speed);

#endif
