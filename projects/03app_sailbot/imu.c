/**
 * @file imu.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief Module for controlling the IMU on Kyosho Fortune 612 SailBot.
 *
 * @copyright Inria, 2022
 *
 */

#include "math.h"
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

// 1 / 6842, where 6842 is sensitivy from the datasheet
#define LIS3MDL_SENSITIVITY_4_GAUSS 0.0146156f

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

    // set ultra-high-performance mode on the X/Y axes, output data rate at 10 Hz
    tmp = 0x70;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1_REG, &tmp, 1);
    // set full scale +- 4 gauss
    tmp = 0x00;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2_REG, &tmp, 1);
    // set continous-measurement mode
    tmp = 0x00;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3_REG, &tmp, 1);
    // set ultra high-performance mode on the Z-axis
    tmp = 0x0c;
    db_i2c_write_regs(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4_REG, &tmp, 1);

    // poll until first data is available
    do {
        db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_STATUS_REG, &tmp, 1);
    } while ((tmp & 0x8) == 0);

    _imu_vars.data_ready = true;

    db_i2c_end();

    // Configure DATARDY GPIO as input and generate an interrupt on rising edge
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

float imu_read_heading() {
    lis3mdl_compass_data_t raw_data;
    uint8_t                tmp;
    float                  x;
    float                  y;

    if (!_imu_vars.data_ready) {
        return 0;
    }

    db_i2c_begin();

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_L_REG, &tmp, 1);
    raw_data.x = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_H_REG, &tmp, 1);
    raw_data.x |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_L_REG, &tmp, 1);
    raw_data.y = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_H_REG, &tmp, 1);
    raw_data.y |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_L_REG, &tmp, 1);
    raw_data.z = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_H_REG, &tmp, 1);
    raw_data.z |= (int16_t)tmp << 8;
    _imu_vars.data_ready = false;
    db_i2c_end();

    // convert to heading

    // convert raw data to uT
    x = (float)raw_data.x * LIS3MDL_SENSITIVITY_4_GAUSS;
    y = (float)raw_data.y * LIS3MDL_SENSITIVITY_4_GAUSS;

    // atan2(x,y) for north-clockwise convention
    return atan2f(x, y);
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
