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
#include "imu.h"
#include "timer_hf.h"

//=========================== defines ==========================================

static const gpio_t scl     = { .port = 1, .pin = 9 };
static const gpio_t sda     = { .port = 0, .pin = 11 };
static const gpio_t mag_int = { .port = 0, .pin = 17 };

#define LIS3MDL_ADDR           (0x1C)
#define LIS3MDL_WHO_AM_I_REG   (0x0F)
#define LIS3MDL_CTRL_REG1_REG  (0x20)
#define LIS3MDL_CTRL_REG2_REG  (0x21)
#define LIS3MDL_CTRL_REG3_REG  (0x22)
#define LIS3MDL_CTRL_REG4_REG  (0x23)
#define LIS3MDL_STATUS_REG     (0x27)
#define LIS3MDL_OUT_X_L_REG    (0x28)
#define LIS3MDL_OUT_X_H_REG    (0x29)
#define LIS3MDL_OUT_Y_L_REG    (0x2a)
#define LIS3MDL_OUT_Y_H_REG    (0x2b)
#define LIS3MDL_OUT_Z_L_REG    (0x2c)
#define LIS3MDL_OUT_Z_H_REG    (0x2d)
#define LIS3MDL_TEMP_OUT_L_REG (0x2e)
#define LIS3MDL_TEMP_OUT_H_REG (0x2f)

#define LIS3MDL_WHO_AM_I_VAL (0x3D)

typedef struct {
    bool data_ready;
} imu_vars_t;

//=========================== variables ========================================

imu_vars_t _imu_vars;

//============================== public ========================================

void imu_init(void) {
    uint8_t who_am_i;
    uint8_t tmp;

    db_i2c_init(&scl, &sda);
    db_i2c_begin();
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_WHO_AM_I_REG, &who_am_i, 1);
    assert(who_am_i == LIS3MDL_WHO_AM_I_VAL);

    // set full scale +- 12 Hz
    tmp = 0x40;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2_REG, &tmp, 1);
    // set HP mode on the X/Y axes, ODR at 10 Hz and activate temperature sensor
    tmp = 0xd0;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1_REG, &tmp, 1);
    // set ultra high-performance mode on the Z-axis
    tmp = 0x0c;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4_REG, &tmp, 1);
    // set continous-measurement mode
    tmp = 0x00;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3_REG, &tmp, 1);

    do {
        db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_STATUS_REG, &tmp, 1);
    } while ((tmp & 0x8) == 0);  // poll until first data is available

    _imu_vars.data_ready = true;

    db_i2c_end();

    NRF_P0->PIN_CNF[mag_int.pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                                   (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                                   (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) |   // Activate the Pull-up resistor
                                   (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);      // Sense for low level

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (mag_int.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (mag_int.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

    NVIC_EnableIRQ(GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}

void imu_read_heading(lis3mdl_compass_data_t *out) {
    uint8_t tmp;

    db_i2c_begin();

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_L_REG, &tmp, 1);
    out->x = tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_H_REG, &tmp, 1);
    out->x |= tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_L_REG, &tmp, 1);
    out->y = tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_H_REG, &tmp, 1);
    out->y |= tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_L_REG, &tmp, 1);
    out->z = tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_H_REG, &tmp, 1);
    out->z |= tmp << 8;

    db_i2c_end();

    _imu_vars.data_ready = false;
}

bool imu_data_ready(void) {
    return _imu_vars.data_ready;
}

//============================== interrupts ====================================

void GPIOTE_IRQHandler(void) {
    if (NRF_GPIOTE->EVENTS_PORT) {
        NRF_GPIOTE->EVENTS_PORT = 0;
        _imu_vars.data_ready    = true;
    }
}
