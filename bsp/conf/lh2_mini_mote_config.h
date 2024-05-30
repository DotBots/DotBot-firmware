#ifndef __BOARD_LH2_MINI_MOTE_CONFIG_H
#define __BOARD_LH2_MINI_MOTE_CONFIG_H

/**
 * @defgroup    bsp_board_config_lh2_mini_mote    lh2_mini_mote
 * @ingroup     bsp_board_config
 * @brief       lh2_mini_mote board configuration
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2024
 * @}
 */

/**
 * @name    Debug pins definitions
 * @{
 */
#define DB_DEBUG1_PORT 0
#define DB_DEBUG1_PIN  28
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
#define DB_BTN1_PORT 0
#define DB_BTN1_PIN  29
/** @} */

/**
 * @name    I2C pins definitions
 * @{
 */
#define DB_I2C_SCL_PORT 0
#define DB_I2C_SCL_PIN  4
#define DB_I2C_SDA_PORT 0
#define DB_I2C_SDA_PIN  31
/** @} */

/**
 * @name    LH2 event and data pins definitions
 * @{
 */
#define DB_LH2_E_PORT 0
#define DB_LH2_E_PIN  17
#define DB_LH2_D_PORT 0
#define DB_LH2_D_PIN  20
/** @} */

/**
 * @name    UART pins definitions
 * @{
 */
#define DB_UART_RX_PORT 0
#define DB_UART_RX_PIN  29
#define DB_UART_TX_PORT DB_DEBUG1_PORT
#define DB_UART_TX_PIN  DB_DEBUG1_PIN
/** @} */

/**
 * @name    Regulator pin definitions
 * @{
 */
#define DB_REGULATOR_PORT 0
#define DB_REGULATOR_PIN  30
/** @} */

/**
 * @name    ISM330 pin definitions
 * @{
 */
#define DB_ISM330_INT1_PORT 0
#define DB_ISM330_INT1_PIN  11
#define DB_ISM330_INT2_PORT 1
#define DB_ISM330_INT2_PIN  9
/** @} */

/**
 * @name    Motor driver pins definitions
 * @{
 */
#define DB_MOTOR_AIN1_PORT 0
#define DB_MOTOR_AIN1_PIN  2
#define DB_MOTOR_AIN2_PORT 0
#define DB_MOTOR_AIN2_PIN  28
#define DB_MOTOR_BIN1_PORT 0
#define DB_MOTOR_BIN1_PIN  3
#define DB_MOTOR_BIN2_PORT 1
#define DB_MOTOR_BIN2_PIN  6  // not connected pin. just used to get the code to compile.
/** @} */

/**
 * @name    RGB LED pwm pins definitions
 * @{
 */
#define DB_RGB_LED_PWM_RED_PORT   0
#define DB_RGB_LED_PWM_RED_PIN    28
#define DB_RGB_LED_PWM_GREEN_PORT 0
#define DB_RGB_LED_PWM_GREEN_PIN  2
#define DB_RGB_LED_PWM_BLUE_PORT  0
#define DB_RGB_LED_PWM_BLUE_PIN   3
/** @} */

#endif
