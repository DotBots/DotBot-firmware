/**
 * @file
 * @ingroup drv_lsm6ds
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Driver for controlling the LSM6DS IMU.
 *
 * @copyright Inria, 2023
 *
 */

#include <assert.h>
#include <stdbool.h>

#include "board_config.h"
#include "gpio.h"
#include "i2c.h"
#include "lsm6ds.h"

//=========================== defines ==========================================

static const gpio_t scl     = { .port = DB_I2C_SCL_PORT, .pin = DB_I2C_SCL_PIN };
static const gpio_t sda     = { .port = DB_I2C_SDA_PORT, .pin = DB_I2C_SDA_PIN };
static const gpio_t imu_int = { .port = DB_LSM6DS_INT_PORT, .pin = DB_LSM6DS_INT_PIN };

#define LSM6DS_ADDR              (0x6a)
#define LSM6DS_WHO_AM_I_REG      (0x0f)
#define LSM6DS_FIFO_CTRL5_REG    (0x0a)
#define LSM6DS_CTRL1_XL_REG      (0x10)
#define LSM6DS_CTRL3_REG         (0x12)
#define LSM6DS_CTRL4_REG         (0x13)
#define LSM6DS_MASTER_CONFIG_REG (0x1a)
#define LSM6DS_INT1_CTRL_REG     (0x0d)
#define LSM6DS_OUTX_L_XL_REG     (0x28)
#define LSM6DS_OUTX_H_XL_REG     (0x29)
#define LSM6DS_OUTY_L_XL_REG     (0x2a)
#define LSM6DS_OUTY_H_XL_REG     (0x2b)
#define LSM6DS_OUTZ_L_XL_REG     (0x2c)
#define LSM6DS_OUTZ_H_XL_REG     (0x2d)
#define LSM6DS_STATUS_REG        (0x1e)

#define LSM6DS_WHO_AM_I_VAL (0x6a)

// sensitivity at +-2g
#define LSM6DS_SENSITIVITY 0.061f

#define I2C_DEV (0)

typedef struct {
    lsm6ds_data_ready_cb_t callback;
    bool                   data_ready;
} lsm6ds_vars_t;

//=========================== variables ========================================

static lsm6ds_vars_t _lsm6ds_vars;

//=========================== prototypes ========================================

void        lsm6ds_i2c_read_accelerometer(lsm6ds_acc_data_t *out);
static void cb_imu_int(void *ctx);

//============================== public ========================================

bool lsm6ds_data_ready(void) {
    return _lsm6ds_vars.data_ready;
}

void lsm6ds_init(lsm6ds_data_ready_cb_t callback) {
    uint8_t           who_am_i;
    uint8_t           tmp;
    lsm6ds_acc_data_t dummy_data;

    _lsm6ds_vars.callback = callback;

    // init the interrupts
    db_gpio_init_irq(&imu_int, DB_GPIO_IN_PD, DB_GPIO_IRQ_EDGE_RISING, cb_imu_int, NULL);

    // init I2C
    db_i2c_init(I2C_DEV, &scl, &sda);

    db_i2c_begin(I2C_DEV);
    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == LSM6DS_WHO_AM_I_VAL);

    // enable accelerometer at 12.5 Hz, +- 2g
    tmp = 0x10;
    db_i2c_write_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_CTRL1_XL_REG, &tmp, 1);

    tmp = 0x10;
    db_i2c_write_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_CTRL4_REG, &tmp, 1);

    tmp = 0x01;
    db_i2c_write_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_INT1_CTRL_REG, &tmp, 1);

    db_i2c_end(I2C_DEV);

    // do a fake read to trigger
    lsm6ds_i2c_read_accelerometer(&dummy_data);
}

void lsm6ds_read_accelerometer(lsm6ds_acc_data_t *out) {

    lsm6ds_i2c_read_accelerometer(out);
    return;
}

void lsm6ds_i2c_read_accelerometer(lsm6ds_acc_data_t *out) {
    uint8_t tmp;

    db_i2c_begin(I2C_DEV);

    // make sure that data is ready for read
    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_STATUS_REG, &tmp, 1);

    if ((tmp & 0x1) == 0) {
        db_i2c_end(I2C_DEV);
        return;
    }

    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTX_L_XL_REG, &tmp, 1);
    out->x = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTX_H_XL_REG, &tmp, 1);
    out->x |= (int16_t)tmp << 8;

    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTY_L_XL_REG, &tmp, 1);
    out->y = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTY_H_XL_REG, &tmp, 1);
    out->y |= (int16_t)tmp << 8;

    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTZ_L_XL_REG, &tmp, 1);
    out->z = (int16_t)tmp;
    db_i2c_read_regs(I2C_DEV, LSM6DS_ADDR, LSM6DS_OUTZ_H_XL_REG, &tmp, 1);
    out->z |= (int16_t)tmp << 8;
    db_i2c_end(I2C_DEV);

    _lsm6ds_vars.data_ready = false;
}

//============================== interrupts ====================================

static void cb_imu_int(void *ctx) {
    (void)ctx;
    // set data_ready to true
    _lsm6ds_vars.data_ready = true;
    //  invoke application callback if initialized
    if (_lsm6ds_vars.callback != NULL) {
        _lsm6ds_vars.callback();
    }
}
