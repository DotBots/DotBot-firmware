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

#include "gpio.h"
#include "i2c.h"
#include "lis2mdl.h"

//=========================== defines ==========================================

static const gpio_t scl     = { .port = 1, .pin = 9 };
static const gpio_t sda     = { .port = 0, .pin = 11 };
static const gpio_t mag_int = { .port = 0, .pin = 17 };
// static const gpio_t button_2 = { .port = 0, .pin = 12 };

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

typedef struct {
    lis2mdl_data_ready_cb_t callback;
    bool                    data_ready;
    float                   heading;
} lis2mdl_vars_t;

//=========================== variables ========================================

static lis2mdl_vars_t _lis2mdl_vars;

//=========================== prototypes ========================================

void lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out);

//============================== public ========================================

void lis2mdl_init(lis2mdl_data_ready_cb_t callback) {
    uint8_t who_am_i;
    uint8_t tmp;

    _lis2mdl_vars.callback = callback;

    db_i2c_init(&scl, &sda);
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

    // Configure DATARDY GPIO as input and generate an interrupt on rising edge
    NRF_P0->PIN_CNF[mag_int.pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                                   (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                                   (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) |   // Activate the Pull-down resistor
                                   (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);      // Sense for high level

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (mag_int.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (mag_int.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

    NVIC_EnableIRQ(GPIOTE_IRQn);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}

// function should be invoked only upon DATA RDY interrupt, outside of the interrupt context
void lis2mdl_i2c_read_magnetometer(lis2mdl_compass_data_t *out) {
    uint8_t tmp;

    db_i2c_begin();

    // make sure that data is ready for read
    db_i2c_read_regs(LIS2MDL_ADDR, LIS2MDL_STATUS_REG, &tmp, 1);
    assert((tmp & 0x8) != 0);

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

void GPIOTE_IRQHandler(void) {
    uint32_t pins;
    pins = NRF_P0->IN;

    if (NRF_GPIOTE->EVENTS_PORT) {
        NRF_GPIOTE->EVENTS_PORT = 0;
        if (pins & GPIO_IN_PIN17_Msk) {  // if pin 17 is high, data is ready
            // set data_ready to true
            _lis2mdl_vars.data_ready = true;
            // invoke application callback if initialized
            if (_lis2mdl_vars.callback != NULL) {
                _lis2mdl_vars.callback();
            }
        }
    }
}
