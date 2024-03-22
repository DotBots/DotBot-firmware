#ifndef __BOARD_FREEBOT_V1_0_CONFIG_H
#define __BOARD_FREEBOT_V1_0_CONFIG_H

/**
 * @defgroup    bsp_board_config_freebot-v1_0    Freebot v1.0
 * @ingroup     bsp_board_config
 * @brief       FreeBot v1.0 board configuration
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

/**
 * @name    Debug pins definitions
 * @{
 */
#define DB_DEBUG1_PORT 0
#define DB_DEBUG1_PIN  7
#define DB_DEBUG2_PORT 0
#define DB_DEBUG2_PIN  12
#define DB_DEBUG3_PORT 1
#define DB_DEBUG3_PIN  9
/** @} */

/**
 * @name    LEDs pins definitions
 * @{
 */
#define DB_LED1_PORT DB_DEBUG1_PORT
#define DB_LED1_PIN  DB_DEBUG1_PIN
/** @} */

/**
 * @name    Buttons pins definitions
 * @{
 */
#define DB_BTN1_PORT DB_DEBUG3_PORT
#define DB_BTN1_PIN  DB_DEBUG3_PIN
/** @} */

/**
 * @name    I2C pins definitions
 * @{
 */
#define DB_I2C_SCL_PORT DB_DEBUG1_PORT
#define DB_I2C_SCL_PIN  DB_DEBUG1_PIN
#define DB_I2C_SDA_PORT DB_DEBUG2_PORT
#define DB_I2C_SDA_PIN  DB_DEBUG2_PIN
/** @} */

/**
 * @name    UART pins definitions
 * @{
 */
#define DB_UART_RX_PORT 0
#define DB_UART_RX_PIN  13
#define DB_UART_TX_PORT 0
#define DB_UART_TX_PIN  24
/** @} */

/**
 * @name    Motor driver pins definitions
 * @{
 */
#define DB_MOTOR_AIN1_PORT 0
#define DB_MOTOR_AIN1_PIN  9
#define DB_MOTOR_BIN1_PORT 0
#define DB_MOTOR_BIN1_PIN  10
#define DB_MOTOR_AIN2_PORT 0
#define DB_MOTOR_AIN2_PIN  17
#define DB_MOTOR_BIN2_PORT 0
#define DB_MOTOR_BIN2_PIN  15
#define DB_MOTOR_AIN3_PORT 1
#define DB_MOTOR_AIN3_PIN  10
#define DB_MOTOR_BIN3_PORT 1
#define DB_MOTOR_BIN3_PIN  11
#define DB_MOTOR_AIN4_PORT 0
#define DB_MOTOR_AIN4_PIN  31
#define DB_MOTOR_BIN4_PORT 0
#define DB_MOTOR_BIN4_PIN  30
/** @} */

/**
 * @name    LSM6DS pin definitions
 * @{
 */
#define DB_LSM6DS_INT_PORT DB_DEBUG3_PORT
#define DB_LSM6DS_INT_PIN  DB_DEBUG3_PIN
/** @} */

/**
 * @name    LIS2MDL pin definitions
 * @{
 */
#define DB_LIS2MDL_INT_PORT DB_DEBUG3_PORT
#define DB_LIS2MDL_INT_PIN  DB_DEBUG3_PIN
/** @} */

#endif
