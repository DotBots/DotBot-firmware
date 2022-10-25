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

static const gpio_t scl      = { .port = 1, .pin = 9 };
static const gpio_t sda      = { .port = 0, .pin = 11 };
static const gpio_t mag_int  = { .port = 0, .pin = 17 };
static const gpio_t button_2 = { .port = 0, .pin = 12 };

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

#define SAILBOT_REV01_OFFSET_X (-142)
#define SAILBOT_REV01_OFFSET_Y (576)
#define SAILBOT_REV01_OFFSET_Z (-326)

// 1 / 6842, where 6842 is sensitivy from the datasheet
#define LIS3MDL_SENSITIVITY_4_GAUSS 0.0146156f

typedef struct {
    bool                   data_ready;
    bool                   calibrate;
    uint32_t               pins;
    lis3mdl_compass_data_t max;
    lis3mdl_compass_data_t min;
} imu_vars_t;

//=========================== variables ========================================

imu_vars_t _imu_vars;

//=========================== prototypes ========================================

void imu_i2c_read_magnetometer(lis3mdl_compass_data_t *out);

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

void imu_i2c_read_magnetometer(lis3mdl_compass_data_t *out) {
    uint8_t tmp;

    if (!_imu_vars.data_ready) {
        return;
    }

    db_i2c_begin();
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_L_REG, &tmp, 1);
    out->x = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_H_REG, &tmp, 1);
    out->x |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_L_REG, &tmp, 1);
    out->y = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Y_H_REG, &tmp, 1);
    out->y |= (int16_t)tmp << 8;

    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_L_REG, &tmp, 1);
    out->z = (int16_t)tmp;
    db_i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_Z_H_REG, &tmp, 1);
    out->z |= (int16_t)tmp << 8;
    _imu_vars.data_ready = false;
    db_i2c_end();
}

void imu_magnetometer_calibrate(float *offset_x, float *offset_y, float *offset_z) {
    lis3mdl_compass_data_t current;
    lis3mdl_compass_data_t max = { 0, 0, 0 };
    lis3mdl_compass_data_t min = { 0, 0, 0 };

    _imu_vars.calibrate = true;

    // Configure BUTTON2 as input and generate an interrupt on falling edge
    NRF_P0->PIN_CNF[button_2.pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Pin as input
                                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Activate the input
                                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |     // Activate the Pull-up resistor
                                    (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);       // Sense for low level

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (button_2.pin << GPIOTE_CONFIG_PSEL_Pos) |
                            (button_2.port << GPIOTE_CONFIG_PORT_Pos) |
                            (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    printf("Starting calibration...\n");
    while (_imu_vars.calibrate) {
        if (imu_data_ready()) {
            // until button is pressed, loop and save max and min values
            imu_i2c_read_magnetometer(&current);
            printf("%d, %d, %d\n", current.x, current.y, current.z);

            if (current.x > max.x) {
                max.x = current.x;
            }
            if (current.x < min.x) {
                min.x = current.x;
            }
            if (current.y > max.y) {
                max.y = current.y;
            }
            if (current.y < min.y) {
                min.x = current.x;
            }
            if (current.z > max.z) {
                max.z = current.z;
            }
            if (current.z < min.z) {
                min.z = current.z;
            }
        }
        __WFE();
    }

    *offset_x = (float)(max.x + min.x) / 2.0;
    *offset_y = (float)(max.y + min.y) / 2.0;
    *offset_z = (float)(max.z + min.z) / 2.0;
    return;
}

float imu_read_heading() {
    lis3mdl_compass_data_t raw_data;
    float                  x;
    float                  y;

    imu_i2c_read_magnetometer(&raw_data);

    // convert to heading

    // convert raw data to uT
    x = (float)(raw_data.x - SAILBOT_REV01_OFFSET_X) * LIS3MDL_SENSITIVITY_4_GAUSS;
    y = (float)(raw_data.y - SAILBOT_REV01_OFFSET_Y) * LIS3MDL_SENSITIVITY_4_GAUSS;

    // atan2(x,y) for north-clockwise convention
    return atan2f(x, y);
}

bool imu_data_ready(void) {
    return _imu_vars.data_ready;
}

//============================== interrupts ====================================

void GPIOTE_IRQHandler(void) {
    uint32_t pins;
    pins = NRF_P0->IN;

    if (NRF_GPIOTE->EVENTS_PORT) {
        NRF_GPIOTE->EVENTS_PORT = 0;
        if (pins & GPIO_IN_PIN17_Msk) {  // if pin 17 is high, data is ready
            _imu_vars.data_ready = true;
        } else if (!(pins & GPIO_IN_PIN12_Msk)) {  // if pin 12 is low, stop calibration
            _imu_vars.calibrate = false;
        }
    }
}
