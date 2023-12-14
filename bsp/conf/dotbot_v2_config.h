#ifndef __BOARD_DOTBOT_V2_CONFIG_H
#define __BOARD_DOTBOT_V2_CONFIG_H

/**
 * @defgroup    bsp_board_config_dotbot-v2    DotBot v2
 * @ingroup     bsp_board_config
 * @brief       DotBot v2 board configuration
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
#define DB_DEBUG1_PIN  6
#define DB_DEBUG2_PORT 0
#define DB_DEBUG2_PIN  7
#define DB_DEBUG3_PORT 0
#define DB_DEBUG3_PIN  20
/** @} */

/**
 * @name    LEDs pins definitions
 * @{
 */
#define DB_LED1_PORT 1
#define DB_LED1_PIN  4
#define DB_LED2_PORT 1
#define DB_LED2_PIN  5
/** @} */

/**
 * @name    Buttons pins definitions
 * @{
 */
#define DB_BTN1_PORT DB_DEBUG1_PORT
#define DB_BTN1_PIN  DB_DEBUG1_PIN
#define DB_BTN2_PORT DB_DEBUG2_PORT
#define DB_BTN2_PIN  DB_DEBUG2_PIN
/** @} */

/**
 * @name    I2C pins definitions
 * @{
 */
#define DB_I2C_SCL_PORT 0
#define DB_I2C_SCL_PIN  2
#define DB_I2C_SDA_PORT 0
#define DB_I2C_SDA_PIN  3
/** @} */

/**
 * @name    UART pins definitions
 * @{
 */
#define DB_UART_RX_PORT DB_DEBUG3_PORT
#define DB_UART_RX_PIN  DB_DEBUG3_PIN
#define DB_UART_TX_PORT 0
#define DB_UART_TX_PIN  25
/** @} */

/**
 * @name    LH2 event and data pins definitions
 * @{
 */
#define DB_LH2_E_PORT 0
#define DB_LH2_E_PIN  27
#define DB_LH2_D_PORT 1
#define DB_LH2_D_PIN  1
/** @} */

/**
 * @name    Regulator pin definitions
 * @{
 */
#define DB_REGULATOR_PORT 0
#define DB_REGULATOR_PIN  8
/** @} */

/**
 * @name    Motor driver pins definitions
 * @{
 */
#define DB_MOTOR_AIN1_PORT 1
#define DB_MOTOR_AIN1_PIN  14
#define DB_MOTOR_AIN2_PORT 1
#define DB_MOTOR_AIN2_PIN  15
#define DB_MOTOR_BIN1_PORT 0
#define DB_MOTOR_BIN1_PIN  29
#define DB_MOTOR_BIN2_PORT 0
#define DB_MOTOR_BIN2_PIN  22
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
 * @name    QDEC pins definitions
 * @{
 */
#define DB_QDEC_LEFT_A_PORT  1
#define DB_QDEC_LEFT_A_PIN   12
#define DB_QDEC_LEFT_B_PORT  1
#define DB_QDEC_LEFT_B_PIN   11
#define DB_QDEC_RIGHT_A_PORT 0
#define DB_QDEC_RIGHT_A_PIN  21
#define DB_QDEC_RIGHT_B_PORT 0
#define DB_QDEC_RIGHT_B_PIN  31
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

/**
 * @name    LIS2MDL pin definitions
 * @{
 */
#define DB_LIS3MDL_DRDY_PORT 1
#define DB_LIS3MDL_DRDY_PIN  2
#define DB_LIS3MDL_INT_PORT  1
#define DB_LIS3MDL_INT_PIN   3
/** @} */

/**
 * @name    RGB LED pwm pins definitions
 * @{
 */
#define DB_RGB_LED_PWM_RED_PORT   0
#define DB_RGB_LED_PWM_RED_PIN    10
#define DB_RGB_LED_PWM_GREEN_PORT 1
#define DB_RGB_LED_PWM_GREEN_PIN  7
#define DB_RGB_LED_PWM_BLUE_PORT  1
#define DB_RGB_LED_PWM_BLUE_PIN   6
/** @} */

#endif
