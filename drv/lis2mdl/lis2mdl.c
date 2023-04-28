/**
 * @file lis2mdl.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for controlling the IMU on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "board_config.h"
#include "gpio.h"
#include "i2c.h"
#include "lis2mdl.h"

//=========================== defines ==========================================

static const gpio_t scl     = { .port = DB_I2C_SCL_PORT, .pin = DB_I2C_SCL_PIN };
static const gpio_t sda     = { .port = DB_I2C_SDA_PORT, .pin = DB_I2C_SDA_PIN };
static const gpio_t mag_int = { .port = 0, .pin = 17 };
static const gpio_t imu_int = { .port = 0, .pin = 20 };

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

typedef struct {
    lis2mdl_data_ready_cb_t callback;
    bool                    data_ready;
    float                   heading;
} lis2mdl_vars_t;

typedef struct {
    lsm6ds_data_ready_cb_t callback;
    bool                   data_ready;
    int16_t                roll;
} lsm6ds_vars_t;

//=========================== variables ========================================

static lis2mdl_vars_t _lis2mdl_vars;
static lsm6ds_vars_t  _lsm6ds_vars;

//=========================== prototypes ========================================

void      lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out);
void      lsm6ds_i2c_read_accelerometer(lsm6ds_acc_data_t *out);
void      interrupt_init(const gpio_t *imu, const gpio_t *magnetometer);
gpio_cb_t cb_mag_int(void *ctx);
gpio_cb_t cb_imu_int(void *ctx);

//============================== public ========================================

void imu9d_init(lsm6ds_data_ready_cb_t accelerometer_callback, lis2mdl_data_ready_cb_t magnetometer_callback) {

    // init the interrupts
    db_gpio_init_irq(&mag_int, DB_GPIO_IN_PD, DB_GPIO_IRQ_EDGE_RISING, cb_mag_int, NULL);
    db_gpio_init_irq(&imu_int, DB_GPIO_IN_PD, DB_GPIO_IRQ_EDGE_RISING, cb_imu_int, NULL);

    // init I2C
    db_i2c_init(&scl, &sda);

    // init the magnetometer sensor
    lis2mdl_init(magnetometer_callback);

    // init the 6D IMU with accelerometer
    lsm6ds_init(accelerometer_callback);
}

int8_t lsm6ds_last_roll(void) {
    return _lsm6ds_vars.roll;
}

bool lsm6ds_data_ready(void) {
    return _lsm6ds_vars.data_ready;
}

void lsm6ds_init(lsm6ds_data_ready_cb_t callback) {
    uint8_t who_am_i;
    uint8_t tmp;

    _lsm6ds_vars.callback = callback;

    // I2C already initialized

    db_i2c_begin();
    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == LSM6DS_WHO_AM_I_VAL);

    // enable accelerometer at 12.5 Hz, +- 2g
    tmp = 0x10;
    db_i2c_write_regs(LSM6DS_ADDR, LSM6DS_CTRL1_XL_REG, &tmp, 1);

    tmp = 0x10;
    db_i2c_write_regs(LSM6DS_ADDR, LSM6DS_CTRL4_REG, &tmp, 1);

    tmp = 0x01;
    db_i2c_write_regs(LSM6DS_ADDR, LSM6DS_INT1_CTRL_REG, &tmp, 1);

    db_i2c_end();

    // do a fake read to trigger
    lsm6ds_read_accelerometer();
}

void lsm6ds_read_accelerometer() {
    lsm6ds_acc_data_t raw_data;

    lsm6ds_i2c_read_accelerometer(&raw_data);

    // convert to roll angle
    _lsm6ds_vars.roll = (int16_t)(atan2f(raw_data.x, raw_data.z) * 180 / M_PI);
    return;
}

void lsm6ds_i2c_read_accelerometer(lsm6ds_acc_data_t *out) {
    uint8_t tmp;

    db_i2c_begin();

    // make sure that data is ready for read
    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_STATUS_REG, &tmp, 1);

    if ((tmp & 0x1) == 0) {
        db_i2c_end();
        return;
    }

    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTX_L_XL_REG, &tmp, 1);
    out->x = (int16_t)tmp;
    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTX_H_XL_REG, &tmp, 1);
    out->x |= (int16_t)tmp << 8;

    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTY_L_XL_REG, &tmp, 1);
    out->y = (int16_t)tmp;
    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTY_H_XL_REG, &tmp, 1);
    out->y |= (int16_t)tmp << 8;

    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTZ_L_XL_REG, &tmp, 1);
    out->z = (int16_t)tmp;
    db_i2c_read_regs(LSM6DS_ADDR, LSM6DS_OUTZ_H_XL_REG, &tmp, 1);
    out->z |= (int16_t)tmp << 8;
    db_i2c_end();

    _lsm6ds_vars.data_ready = false;
}

void lis2mdl_init(lis2mdl_data_ready_cb_t callback) {
    uint8_t who_am_i;
    uint8_t tmp;

    _lis2mdl_vars.callback = callback;

    db_i2c_begin();
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == LIS2MDL_WHO_AM_I_VAL);

    // set continous mode, output data rate at 10 Hz
    tmp = 0x00;
    db_i2c_write_regs(LIS2MDL_ADDR, LIS2MDL_CFG_REG_A_REG, &tmp, 1);
    // keep default values of REG_B
    tmp = 0x00;
    db_i2c_write_regs(LIS2MDL_ADDR, LIS2MDL_CFG_REG_B_REG, &tmp, 1);
    // set DRDY_on_PIN bit
    tmp = 0x01;
    db_i2c_write_regs(LIS2MDL_ADDR, LIS2MDL_CFG_REG_C_REG, &tmp, 1);

    db_i2c_end();

    lis2mdl_read_heading();
}

// function should be invoked only upon DATA RDY interrupt, outside of the interrupt context
void lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out) {
    uint8_t tmp;

    db_i2c_begin();

    // make sure that data is ready for read
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_STATUS_REG, &tmp, 1);

    if ((tmp & 0x8) == 0) {
        db_i2c_end();
        return;
    }

    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTX_L_REG, &tmp, 1);
    out->x = (int16_t)tmp;
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTX_H_REG, &tmp, 1);
    out->x |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTY_L_REG, &tmp, 1);
    out->y = (int16_t)tmp;
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTY_H_REG, &tmp, 1);
    out->y |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTZ_L_REG, &tmp, 1);
    out->z = (int16_t)tmp;
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_OUTZ_H_REG, &tmp, 1);
    out->z |= (int16_t)tmp << 8;
    db_i2c_end();

    _lis2mdl_vars.data_ready = false;
}

void lis2mdl_magnetometer_calibrate(lis2mdl_compass_data_t *offset) {
    (void)offset;
    lis2mdl_compass_data_t current;

    printf("X,Y,Z\n");

    // loop forever
    while (1) {

        // save max and min values
        if (lis2mdl_data_ready()) {
            lis2mdl_i2c_read_magnetometer(&current);
            printf("%d,%d,%d\n", current.x, current.y, current.z);
        }
        __WFE();
    }

    return;
}

void lis2mdl_read_heading(void) {
    lis2mdl_compass_data_t raw_data;
    float                  x;
    float                  y;

    lis2mdl_i2c_read_magnetometer(&raw_data);

    // convert to heading

    // convert raw data to uT and account for offset
    x = (float)(raw_data.x - SAILBOT_REV10_OFFSET_X) * LIS2MDL_SENSITIVITY;
    y = (float)(raw_data.y - SAILBOT_REV10_OFFSET_Y) * LIS2MDL_SENSITIVITY;

    // atan2(x,y) for north-clockwise convention, + Pi for 0 to 2PI heading
    _lis2mdl_vars.heading = atan2f(x, y) + M_PI;
    return;
}

float lis2mdl_last_heading(void) {
    return _lis2mdl_vars.heading;
}

bool lis2mdl_data_ready(void) {
    return _lis2mdl_vars.data_ready;
}

//============================== interrupts ====================================
gpio_cb_t cb_mag_int(void *ctx) {
    // set data_ready to true
    _lis2mdl_vars.data_ready = true;
    // invoke application callback if initialized
    if (_lis2mdl_vars.callback != NULL) {
        _lis2mdl_vars.callback();
    }
}

gpio_cb_t cb_imu_int(void *ctx) {
    // set data_ready to true
    _lsm6ds_vars.data_ready = true;
    //  invoke application callback if initialized
    if (_lsm6ds_vars.callback != NULL) {
        _lsm6ds_vars.callback();
    }
}