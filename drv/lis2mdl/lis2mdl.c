/**
 * @file
 * @ingroup drv_lis2mdl
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for reading the LIS2MDL magnetometer.
 *
 * @copyright Inria, 2022
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdbool.h>

#include "board_config.h"
#include "gpio.h"
#include "i2c.h"
#include "lis2mdl.h"

//=========================== defines ==========================================

static const gpio_t scl     = { .port = DB_I2C_SCL_PORT, .pin = DB_I2C_SCL_PIN };
static const gpio_t sda     = { .port = DB_I2C_SDA_PORT, .pin = DB_I2C_SDA_PIN };
static const gpio_t mag_int = { .port = DB_LIS2MDL_INT_PORT, .pin = DB_LIS2MDL_INT_PIN };

#define LIS2MDL_ADDR           (0x1E)
#define LIS2MDL_OFFSET_X_REG_L (0x45)
#define LIS2MDL_OFFSET_X_REG_H (0x46)
#define LIS2MDL_OFFSET_Y_REG_L (0x47)
#define LIS2MDL_OFFSET_Y_REG_H (0x48)
#define LIS2MDL_OFFSET_Z_REG_L (0x49)
#define LIS2MDL_OFFSET_Z_REG_H (0x4a)
#define LIS2MDL_WHO_AM_I_REG   (0x4F)
#define LIS2MDL_CFG_REG_A_REG  (0x60)
#define LIS2MDL_CFG_REG_B_REG  (0x61)
#define LIS2MDL_CFG_REG_C_REG  (0x62)
#define LIS2MDL_INT_CRTL_REG   (0x63)
#define LIS2MDL_INT_SOURCE_REG (0x64)
#define LIS2MDL_STATUS_REG     (0x67)
#define LIS2MDL_OUTX_L_REG     (0x68)
#define LIS2MDL_OUTX_H_REG     (0x69)
#define LIS2MDL_OUTY_L_REG     (0x6a)
#define LIS2MDL_OUTY_H_REG     (0x6b)
#define LIS2MDL_OUTZ_L_REG     (0x6c)
#define LIS2MDL_OUTZ_H_REG     (0x6d)

#define LIS2MDL_WHO_AM_I_VAL (0x40)

#define SAILBOT_REV10_OFFSET_X (-273)
#define SAILBOT_REV10_OFFSET_Y (160)
#define SAILBOT_REV10_OFFSET_Z (367)

// 1 / 6842, where 6842 is sensitivy from the datasheet
#define LIS2MDL_SENSITIVITY 1.5f

#define I2C_DEV (0)

typedef struct {
    lis2mdl_data_ready_cb_t callback;
    bool                    data_ready;
} lis2mdl_vars_t;

//=========================== variables ========================================

static lis2mdl_vars_t _lis2mdl_vars;

//=========================== prototypes ========================================

void        lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out);
static void cb_mag_int(void *ctx);

//============================== public ========================================

void lis2mdl_init(lis2mdl_data_ready_cb_t callback) {
    uint8_t                who_am_i;
    uint8_t                tmp;
    lis2mdl_compass_data_t dummy_data;

    _lis2mdl_vars.callback = callback;

    // init the interrupts
    db_gpio_init_irq(&mag_int, DB_GPIO_IN_PD, DB_GPIO_IRQ_EDGE_RISING, cb_mag_int, NULL);

    // init I2C
    db_i2c_init(I2C_DEV, &scl, &sda);
    db_i2c_begin(I2C_DEV);
    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == LIS2MDL_WHO_AM_I_VAL);

    // set continous mode, output data rate at 10 Hz
    tmp = 0x00;
    db_i2c_write_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_CFG_REG_A_REG, &tmp, 1);
    // keep default values of REG_B
    tmp = 0x00;
    db_i2c_write_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_CFG_REG_B_REG, &tmp, 1);
    // set DRDY_on_PIN bit
    tmp = 0x01;
    db_i2c_write_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_CFG_REG_C_REG, &tmp, 1);

    db_i2c_end(I2C_DEV);

    // trigger dummy read of data
    lis2mdl_i2c_read_magnetometer(&dummy_data);
}

// function should be invoked only upon DATA RDY interrupt, outside of the interrupt context
void lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out) {
    uint8_t tmp;

    db_i2c_begin(I2C_DEV);

    // make sure that data is ready for read
    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_STATUS_REG, &tmp, 1);

    if ((tmp & 0x8) == 0) {
        db_i2c_end(I2C_DEV);
        return;
    }

    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTX_L_REG, &tmp, 1);
    out->x = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTX_H_REG, &tmp, 1);
    out->x |= (int16_t)tmp << 8;

    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTY_L_REG, &tmp, 1);
    out->y = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTY_H_REG, &tmp, 1);
    out->y |= (int16_t)tmp << 8;

    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTZ_L_REG, &tmp, 1);
    out->z = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LIS2MDL_ADDR, LIS2MDL_OUTZ_H_REG, &tmp, 1);
    out->z |= (int16_t)tmp << 8;
    db_i2c_end(I2C_DEV);

    // compensate for hard-iron offsets
    out->x -= SAILBOT_REV10_OFFSET_X;
    out->y -= SAILBOT_REV10_OFFSET_Y;
    out->z -= SAILBOT_REV10_OFFSET_Z;

    _lis2mdl_vars.data_ready = false;
}

void lis2mdl_read_magnetometer(lis2mdl_compass_data_t *out) {
    lis2mdl_i2c_read_magnetometer(out);

    return;
}

bool lis2mdl_data_ready(void) {
    return _lis2mdl_vars.data_ready;
}

//============================== interrupts ====================================

static void cb_mag_int(void *ctx) {
    (void)ctx;
    // set data_ready to true
    _lis2mdl_vars.data_ready = true;
    // invoke application callback if initialized
    if (_lis2mdl_vars.callback != NULL) {
        _lis2mdl_vars.callback();
    }
}
