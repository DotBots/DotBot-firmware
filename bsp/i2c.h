#ifndef __I2C_H
#define __I2C_H

/**
 * @file i2c.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "i2c" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"

//=========================== public ===========================================

void db_i2c_init(const gpio_t *scl, const gpio_t *sda);
void db_i2c_begin(void);
void db_i2c_end(void);
void db_i2c_read_regs(uint8_t addr, uint8_t reg, void *data, size_t len);
void db_i2c_write_regs(uint8_t addr, uint8_t reg, const void *data, size_t len);

#endif
