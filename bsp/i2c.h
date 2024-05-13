#ifndef __I2C_H
#define __I2C_H

/**
 * @defgroup    bsp_i2c I2C
 * @ingroup     bsp
 * @brief       Control the I2C peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"

typedef uint8_t i2c_t;  ///< I2C peripheral index

/**
 * @brief Initialize the I2C peripheral
 *
 * @param[in]   i2c     i2c reference used
 * @param[in]   scl     pointer to struct that represents the SCL pin
 * @param[in]   sda     pointer to struct that represents the SDA pin
 */
void db_i2c_init(i2c_t i2c, const gpio_t *scl, const gpio_t *sda);

/**
 * @brief Begin transmission on I2C
 *
 * @param[in]   i2c     i2c reference used
 */
void db_i2c_begin(i2c_t i2c);

/**
 * @brief End transmission on I2C
 *
 * @param[in]   i2c     i2c reference used
 */
void db_i2c_end(i2c_t i2c);

/**
 * @brief Read bytes from one register
 *
 * @param[in]   i2c     i2c reference used
 * @param[in]   addr    Address of the device on the I2C bus
 * @param[in]   reg     Address of the register to read
 * @param[out]  data    Pointer to the output byte array
 * @param[in]   len     Length of the bytes to read
 */
void db_i2c_read_regs(i2c_t i2c, uint8_t addr, uint8_t reg, void *data, size_t len);

/**
 * @brief Write bytes to register
 *
 * @param[in]   i2c     i2c reference used
 * @param[in]   addr    Address of the device on the I2C bus
 * @param[in]   reg     Address of the register to write
 * @param[out]  data    Pointer to the input byte array
 * @param[in]   len     Length of the bytes to write
 */
void db_i2c_write_regs(i2c_t i2c, uint8_t addr, uint8_t reg, const void *data, size_t len);

#endif
