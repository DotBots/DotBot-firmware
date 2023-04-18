#ifndef __BOARD_NRF52840DK_CONFIG_H
#define __BOARD_NRF52840DK_CONFIG_H

/**
 * @file nrf52840dk_config.h
 * @addtogroup BSP
 *
 * @brief  nRF52840DK board specific definitions.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

/**
 * @name    LEDs pins definitions
 * @{
 */
#define DB_MOTOR_LED1_PORT 0
#define DB_MOTOR_LED1_PIN  13
#define DB_MOTOR_LED2_PORT 0
#define DB_MOTOR_LED2_PIN  14
#define DB_MOTOR_LED3_PORT 0
#define DB_MOTOR_LED3_PIN  15
#define DB_MOTOR_LED4_PORT 0
#define DB_MOTOR_LED4_PIN  16
/** @} */

/**
 * @name    Buttons pins definitions
 * @{
 */
#define DB_MOTOR_BTN1_PORT 0
#define DB_MOTOR_BTN1_PIN  11
#define DB_MOTOR_BTN2_PORT 0
#define DB_MOTOR_BTN2_PIN  12
#define DB_MOTOR_BTN3_PORT 0
#define DB_MOTOR_BTN3_PIN  24
#define DB_MOTOR_BTN4_PORT 0
#define DB_MOTOR_BTN4_PIN  25
/** @} */

/**
 * @name    Motor driver pins definitions
 * @{
 */
#define DB_MOTOR_AIN1_PORT DB_MOTOR_LED1_PORT
#define DB_MOTOR_AIN1_PIN  DB_MOTOR_LED1_PIN
#define DB_MOTOR_AIN2_PORT DB_MOTOR_LED2_PORT
#define DB_MOTOR_AIN2_PIN  DB_MOTOR_LED2_PIN
#define DB_MOTOR_BIN1_PORT DB_MOTOR_LED3_PORT
#define DB_MOTOR_BIN1_PIN  DB_MOTOR_LED3_PIN
#define DB_MOTOR_BIN2_PORT DB_MOTOR_LED4_PORT
#define DB_MOTOR_BIN2_PIN  DB_MOTOR_LED4_PIN
/** @} */

/**
 * @name    Magnetic encoders pins definitions
 * @{
 */
#define DB_RPM_LEFT_PORT  DB_MOTOR_BTN1_PORT
#define DB_RPM_LEFT_PIN   DB_MOTOR_BTN1_PIN
#define DB_RPM_RIGHT_PORT DB_MOTOR_BTN2_PORT
#define DB_RPM_RIGHT_PIN  DB_MOTOR_BTN2_PIN
/** @} */

#endif
