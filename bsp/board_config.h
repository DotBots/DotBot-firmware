#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

/**
 * @file board_config.h
 * @addtogroup BSP
 *
 * @brief  Board specific definitions.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include "gpio.h"

#if defined(BOARD_DOTBOT_V1)
#include "conf/dotbot_v1_config.h"
#elif defined(BOARD_SAILBOT_V1)
#include "conf/sailbot_v1_config.h"
#elif defined(BOARD_NRF52840DK)
#include "conf/nrf52840dk_config.h"
#elif defined(BOARD_NRF52833DK)
#include "conf/nrf52833dk_config.h"
#elif defined(BOARD_NRF5340DK)
#include "conf/nrf5340dk_config.h"
#else
#error "Unsupported board"
#endif

static const gpio_t db_motors_pins[] = {
    { .port = DB_MOTOR_AIN1_PORT, .pin = DB_MOTOR_AIN1_PIN },
    { .port = DB_MOTOR_AIN2_PORT, .pin = DB_MOTOR_AIN2_PIN },
    { .port = DB_MOTOR_BIN1_PORT, .pin = DB_MOTOR_BIN1_PIN },
    { .port = DB_MOTOR_BIN2_PORT, .pin = DB_MOTOR_BIN2_PIN },
};

static const gpio_t db_rpm_left_pin = {
    .port = DB_RPM_LEFT_PORT, .pin = DB_RPM_LEFT_PORT
};

static const gpio_t db_rpm_right_pin = {
    .port = DB_RPM_RIGHT_PORT, .pin = DB_RPM_RIGHT_PORT
};

#endif
