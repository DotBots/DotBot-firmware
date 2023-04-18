#ifndef __BOARD_DOTBOT_V1_CONFIG_H
#define __BOARD_DOTBOT_V1_CONFIG_H

/**
 * @file dotbot_v1_config.h
 * @addtogroup BSP
 *
 * @brief  DotBot v1 board specific definitions.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

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

#endif
