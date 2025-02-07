#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

/**
 * @defgroup    bsp_board_config    Board configuration
 * @ingroup     bsp_board
 * @brief       Board specific definitions
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include "gpio.h"

#if defined(BOARD_DOTBOT_V1)
#include "conf/dotbot_v1_config.h"
#elif defined(BOARD_DOTBOT_V2)
#include "conf/dotbot_v2_config.h"
#elif defined(BOARD_DOTBOT_V3)
#include "conf/dotbot_v3_config.h"
#elif defined(BOARD_FREEBOT_V1_0)
#include "conf/freebot_v1_0_config.h"
#elif defined(BOARD_SAILBOT_V1)
#include "conf/sailbot_v1_config.h"
#elif defined(BOARD_LH2_MINI_MOTE)
#include "conf/lh2_mini_mote_config.h"
#elif defined(BOARD_NRF52840DK)
#include "conf/nrf52840dk_config.h"
#elif defined(BOARD_NRF52833DK)
#include "conf/nrf52833dk_config.h"
#elif defined(BOARD_NRF5340DK)
#include "conf/nrf5340dk_config.h"
#else
#error "Unsupported board"
#endif

///! LED1 pin
static const gpio_t db_led1 = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };

#if defined(DB_LED2_PORT)
///! LED2 pin
static const gpio_t db_led2 = { .port = DB_LED2_PORT, .pin = DB_LED2_PIN };
#endif

#if defined(DB_LED3_PORT)
///! LED3 pin
static const gpio_t db_led3 = { .port = DB_LED3_PORT, .pin = DB_LED3_PIN };
#endif

#if defined(DB_LED4_PORT)
///! LED4 pin
static const gpio_t db_led4 = { .port = DB_LED4_PORT, .pin = DB_LED4_PIN };
#endif

///! BTN1 pin
static const gpio_t db_btn1 = { .port = DB_BTN1_PORT, .pin = DB_BTN1_PIN };

#if defined(DB_BTN2_PORT)
///! BTN2 pin
static const gpio_t db_btn2 = { .port = DB_BTN2_PORT, .pin = DB_BTN2_PIN };
#endif

#if defined(DB_BTN3_PORT)
///! BTN3 pin
static const gpio_t db_btn3 = { .port = DB_BTN3_PORT, .pin = DB_BTN3_PIN };
#endif

#if defined(DB_BTN4_PORT)
///! BTN4 pin
static const gpio_t db_btn4 = { .port = DB_BTN4_PORT, .pin = DB_BTN4_PIN };
#endif

///! I2C SCL pin
static const gpio_t db_scl = { .port = DB_I2C_SCL_PORT, .pin = DB_I2C_SCL_PIN };

///! I2C SDA pin
static const gpio_t db_sda = { .port = DB_I2C_SDA_PORT, .pin = DB_I2C_SDA_PIN };

///! UART RX pin
static const gpio_t db_uart_rx = { .port = DB_UART_RX_PORT, .pin = DB_UART_RX_PIN };

///! UART TX pin
static const gpio_t db_uart_tx = { .port = DB_UART_TX_PORT, .pin = DB_UART_TX_PIN };

#ifdef DB_LH2_E_PORT
///! LH2 event gpio
static const gpio_t db_lh2_e = { .port = DB_LH2_E_PORT, .pin = DB_LH2_E_PIN };
#endif

#ifdef DB_LH2_D_PORT
///! LH2 data gpio
static const gpio_t db_lh2_d = { .port = DB_LH2_D_PORT, .pin = DB_LH2_D_PIN };
#endif

///! Motor driver pins
static const gpio_t db_motors_pins[] = {
    { .port = DB_MOTOR_AIN1_PORT, .pin = DB_MOTOR_AIN1_PIN },
    { .port = DB_MOTOR_AIN2_PORT, .pin = DB_MOTOR_AIN2_PIN },
    { .port = DB_MOTOR_BIN1_PORT, .pin = DB_MOTOR_BIN1_PIN },
    { .port = DB_MOTOR_BIN2_PORT, .pin = DB_MOTOR_BIN2_PIN },
};

#ifdef DB_RPM_LEFT_PORT
///! Left wheel encoder pin
static const gpio_t db_rpm_left_pin = {
    .port = DB_RPM_LEFT_PORT, .pin = DB_RPM_LEFT_PORT
};
#endif

#ifdef DB_RPM_RIGHT_PORT
///! Right wheel encoder pin
static const gpio_t db_rpm_right_pin = {
    .port = DB_RPM_RIGHT_PORT, .pin = DB_RPM_RIGHT_PORT
};
#endif

#ifdef DB_QDEC_LEFT_A_PORT
///! Left wheel encoder pin A
static const gpio_t db_qdec_left_a_pin = {
    .port = DB_QDEC_LEFT_A_PORT, .pin = DB_QDEC_LEFT_A_PIN
};

///! Left wheel encoder pin B
static const gpio_t db_qdec_left_b_pin = {
    .port = DB_QDEC_LEFT_B_PORT, .pin = DB_QDEC_LEFT_B_PIN
};

///! Right wheel encoder pin A
static const gpio_t db_qdec_right_a_pin = {
    .port = DB_QDEC_RIGHT_A_PORT, .pin = DB_QDEC_RIGHT_A_PIN
};

///! Right wheel encoder pin B
static const gpio_t db_qdec_right_b_pin = {
    .port = DB_QDEC_RIGHT_B_PORT, .pin = DB_QDEC_RIGHT_B_PIN
};
#endif

#endif
