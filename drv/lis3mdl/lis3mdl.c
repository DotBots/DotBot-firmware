/**
 * @file
 * @ingroup drv_lis3mdl
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Module for reading the LIS3MDL magnetometer.
 *
 * @copyright Inria, 2022
 *
 */

#include <assert.h>
#include <stdbool.h>

#include "gpio.h"
#include "i2c.h"
#include "lis3mdl.h"

//=========================== defines ==========================================

// Sensor address on I2C bus
#define LIS3MDL_ADDR (0x1C)

// Registers addresses
#define LIS3MDL_OFFSET_X_REG_L (0x05)
#define LIS3MDL_OFFSET_X_REG_H (0x06)
#define LIS3MDL_OFFSET_Y_REG_L (0x07)
#define LIS3MDL_OFFSET_Y_REG_H (0x08)
#define LIS3MDL_OFFSET_Z_REG_L (0x09)
#define LIS3MDL_OFFSET_Z_REG_H (0x0a)
#define LIS3MDL_WHO_AM_I_REG   (0x0F)
#define LIS3MDL_CTRL_REG1      (0x20)
#define LIS3MDL_CTRL_REG2      (0x21)
#define LIS3MDL_CTRL_REG3      (0x22)
#define LIS3MDL_CTRL_REG4      (0x23)
#define LIS3MDL_CTRL_REG5      (0x24)
#define LIS3MDL_STATUS         (0x27)
#define LIS3MDL_OUTX_L         (0x28)
#define LIS3MDL_OUTX_H         (0x29)
#define LIS3MDL_OUTY_L         (0x2a)
#define LIS3MDL_OUTY_H         (0x2b)
#define LIS3MDL_OUTZ_L         (0x2c)
#define LIS3MDL_OUTZ_H         (0x2d)
#define LIS3MDL_TEMP_OUT_L     (0x2e)
#define LIS3MDL_TEMP_OUT_H     (0x2f)
#define LIS3MDL_INT_CFG        (0x30)
#define LIS3MDL_INT_SRC        (0x31)
#define LIS3MDL_INT_THS_L      (0x32)
#define LIS3MDL_INT_THS_H      (0x33)

// Constants
#define LIS3MDL_WHO_AM_I_VAL       (0x3D)
#define LIS3MDL_TEMPERATURE_OFFSET (25U)

// Bit fields
#define LIS3MDL_MASK_REG1_TEMP_EN        (0x80)
#define LIS3MDL_MASK_REG2_REBOOT         (0x08)
#define LIS3MDL_MASK_REG2_SOFT_RST       (0x04)
#define LIS3MDL_MASK_REG3_LOW_POWER_EN   (0x02)
#define LIS3MDL_MASK_REG3_CONT_CONV_MODE (0x00)
#define LIS3MDL_MASK_REG3_Z_LOW_POWER    (0x00)
#define LIS3MDL_MASK_REG3_Z_MEDIUM_POWER (0x04)
#define LIS3MDL_MASK_REG5_BDU            (0x40)
#define LIS3MDL_MASK_REG5_BDU_OFF        (0x00)
#define LIS3MDL_MASK_INT_CFG_XIEN        (0x80)
#define LIS3MDL_MASK_INT_CFG_YIEN        (0x40)
#define LIS3MDL_MASK_INT_CFG_ZIEN        (0x20)
#define LIS3MDL_MASK_INT_CFG_IEA         (0x04)
#define LIS3MDL_MASK_INT_CFG_LIR         (0x02)
#define LIS3MDL_MASK_INT_CFG_IEN         (0x01)
#define LIS3MDL_MASK_INT_SRC_PTH_X       (0x80)
#define LIS3MDL_MASK_INT_SRC_PTH_Y       (0x40)
#define LIS3MDL_MASK_INT_SRC_PTH_Z       (0x20)
#define LIS3MDL_MASK_INT_SRC_NTH_X       (0x10)
#define LIS3MDL_MASK_INT_SRC_NTH_Y       (0x08)
#define LIS3MDL_MASK_INT_SRC_NTH_Z       (0x04)
#define LIS3MDL_MASK_INT_SRC_MROI        (0x02)
#define LIS3MDL_MASK_INT_SRC_INT         (0x01)

#define I2C_DEV (0)

typedef struct {
    const gpio_t *drdy_pin;
} lis3mdl_vars_t;

//=========================== variables ========================================

static lis3mdl_vars_t _lis3mdl_vars = { 0 };

//============================== public ========================================

void lis3mdl_init(const lis3mdl_conf_t *conf) {
    uint8_t tmp            = 0;
    _lis3mdl_vars.drdy_pin = conf->mag_drdy;

    // init the drdy pin for input read
    db_gpio_init(conf->mag_drdy, DB_GPIO_IN_PD);

    // init I2C
    db_i2c_init(I2C_DEV, conf->scl, conf->sda);
    db_i2c_begin(I2C_DEV);

    db_i2c_read_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_WHO_AM_I_REG, &tmp, 1);
    assert(tmp == LIS3MDL_WHO_AM_I_VAL);

    // enable temperature sensor, set x/y operation mode and set output data rate
    tmp = LIS3MDL_MASK_REG1_TEMP_EN | conf->xy_mode | conf->odr;
    db_i2c_write_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, &conf->scale, 1);
    // set scale
    db_i2c_write_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, &tmp, 1);
    // set conversion mode
    tmp = conf->z_mode | conf->op_mode;
    db_i2c_write_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, &tmp, 1);
    /* set z-axis operative mode */
    db_i2c_write_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, &conf->z_mode, 1);

    db_i2c_end(I2C_DEV);

    while (!lis3mdl_data_ready()) {}
}

bool lis3mdl_data_ready(void) {
    return db_gpio_read(_lis3mdl_vars.drdy_pin) == 1;
}

void lis3mdl_read_magnetometer(lis3mdl_data_t *out) {
    uint8_t tmp[2] = { 0 };

    db_i2c_begin(I2C_DEV);
    db_i2c_read_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_OUTX_L, &tmp, 2);
    out->x = (tmp[1] << 8) | tmp[0];

    db_i2c_read_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_OUTY_L, &tmp, 2);
    out->y = (tmp[1] << 8) | tmp[0];

    db_i2c_read_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_OUTZ_L, &tmp, 2);
    out->z = (tmp[1] << 8) | tmp[0];
    db_i2c_end(I2C_DEV);
}

void lis3mdl_read_temperature(int16_t *temperature) {
    uint16_t temp = 0;
    db_i2c_begin(I2C_DEV);
    db_i2c_read_regs(I2C_DEV, LIS3MDL_ADDR, LIS3MDL_TEMP_OUT_L, &temp, 2);
    temp /= 256;
    *temperature = LIS3MDL_TEMPERATURE_OFFSET + temp;
    db_i2c_end(I2C_DEV);
}
