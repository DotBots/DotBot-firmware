#ifndef __BOARD_DOTBOT_V1_CONFIG_H
#define __BOARD_DOTBOT_V1_CONFIG_H

/**
 * @defgroup    bsp_board_config_dotbot-v1    DotBot v1
 * @ingroup     bsp_board_config
 * @brief       DotBot v1 board configuration
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
#define DB_DEBUG1_PIN  9
#define DB_DEBUG2_PORT 0
#define DB_DEBUG2_PIN  10
#define DB_DEBUG3_PORT 0
#define DB_DEBUG3_PIN  31
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
#define DB_BTN1_PORT DB_DEBUG2_PORT
#define DB_BTN1_PIN  DB_DEBUG2_PIN
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
#define DB_UART_RX_PORT DB_DEBUG1_PORT
#define DB_UART_RX_PIN  DB_DEBUG1_PIN
#define DB_UART_TX_PORT DB_DEBUG2_PORT
#define DB_UART_TX_PIN  DB_DEBUG2_PIN
/** @} */

/**
 * @name    LH2 event and data pins definitions
 * @{
 */
#define DB_LH2_E_PORT 0
#define DB_LH2_E_PIN  30
#define DB_LH2_D_PORT 0
#define DB_LH2_D_PIN  29
/** @} */

/**
 * @name    Regulator pin definitions
 * @{
 */
#define DB_REGULATOR_PORT 0
#define DB_REGULATOR_PIN  20
/** @} */

/**
 * @name    Motor driver pins definitions
 * @{
 */
#define DB_MOTOR_AIN1_PORT 0
#define DB_MOTOR_AIN1_PIN  2
#define DB_MOTOR_AIN2_PORT 0
#define DB_MOTOR_AIN2_PIN  28
#define DB_MOTOR_BIN1_PORT 1
#define DB_MOTOR_BIN1_PIN  9
#define DB_MOTOR_BIN2_PORT 0
#define DB_MOTOR_BIN2_PIN  11
/** @} */

/**
 * @name    Magnetic encoders pins definitions
 * @{
 */
#define DB_RPM_LEFT_PORT  0
#define DB_RPM_LEFT_PIN   17
#define DB_RPM_RIGHT_PORT 0
#define DB_RPM_RIGHT_PIN  15
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
