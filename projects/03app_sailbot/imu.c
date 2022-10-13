/**
 * @file imu.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for controlling the IMU on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */

#include "gpio.h"

static const gpio_t scl = { .port = 1, .pin = 9 };
static const gpio_t sda = { .port = 0, .pin = 11 };

#define ICM20948_ADDR         (0x68)
#define ICM20948_WHO_AM_I_REG (0x00)
#define ICM20948_WHO_AM_I_VAL (0xEA)

void imu_init(void) {

    db_i2c_init(&scl, &sda);
    db_i2c_begin();
    uint8_t who_am_i;
    db_i2c_read_regs(ICM20948_ADDR, ICM20948_WHO_AM_I_REG, &who_am_i, 1);
    if (who_am_i != ICM20948_WHO_AM_I_VAL) {
        printf(
            "Invalid WHO_AM_I value '%d' (expected: %d)\n",
            who_am_i, ICM20948_WHO_AM_I_VAL);
        return;
    } else {
        printf("Valid WHO_AM_I value\n");
    }
}