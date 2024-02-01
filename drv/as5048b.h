#ifndef __AS5048B_H
#define __AS5048B_H

/**
 * @defgroup    drv_as5048b  AS5048B rotary encoder driver
 * @ingroup     drv
 * @brief       Driver for the AS5048B rotary encoder
 *
 * @{
 * @file
 * @author      Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * @copyright   Inria, 2024
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

/**
 * @brief Initialise the AS5048B rotary encoder
 */
void as5048b_init(void);

/**
 * @brief Reads 14-bit raw absolute angle data on AS5048B over I2C
 *
 * @param[out] angle_uint_out 14-bit raw angle (0x0 to 0x3FFF)
 */
void as5048b_i2c_read_raw_angle(uint16_t *angle_uint_out);

/**
 * @brief Reads angle as a float in radians
 *
 * @param[out] angle_rad_out Float angle [0, 2*M_PI)
 */
void as5048b_i2c_read_angle_radian(float *angle_rad_out);

/**
 * @brief Reads angle as a float in degrees
 *
 * @param[out] angle_deg_out Float angle [0, 360.)
 */
void as5048b_i2c_read_angle_degree(float *angle_deg_out);

/**
 * @brief Convert the raw angle to interval [0, max_angle)
 *
 * @param[in] raw_angle 14-bit raw angle (0x0 to 0x3FFF)
 * @param[in] max_angle Maximum angle for conversion
 *
 * @return Float angle in interval [0, max_angle)
 */
float as5048b_convert_raw_angle(uint16_t raw_angle, float max_angle);

#endif
