/**
 * @file imu.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for controlling the IMU on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */

#include "gpio.h"
#include "assert.h"
#include "i2c.h"

static const gpio_t scl = { .port = 1, .pin = 9 };
static const gpio_t sda = { .port = 0, .pin = 11 };

#define ICM20948_ADDR         (0x68)
#define ICM20948_WHO_AM_I_REG (0x00)
#define ICM20948_WHO_AM_I_VAL (0xEA)
#define ICM20948_BANK_SEL_REG (0x7f)

#define AK09916_ADDR (0x0C)
#define AK09916_WIA1_REG (0x00)
#define AK09916_WIA1_VAL (0x48)

void imu_init(void) {
    uint8_t who_am_i;

    db_i2c_init(&scl, &sda);
    db_i2c_begin();
    db_i2c_read_regs(ICM20948_ADDR, ICM20948_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == ICM20948_WHO_AM_I_VAL);

    db_i2c_end();
}
