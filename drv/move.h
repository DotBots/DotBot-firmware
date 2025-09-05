#ifndef __MOVE_H
#define __MOVE_H

/**
 * @defgroup    drv_move    High level move commands for DotBot
 * @ingroup     drv
 * @brief       High level move commands for DotBot: move straight, rotate
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>

//=========================== public ===========================================

/**
 * @brief   Initialize qdec and motors
 */
void db_move_init();

/**
 * @brief   Deinitialize the move module
 *
 * Only stop the move timer
 */
void db_move_deinit(void);

/**
 * @brief   Move straight by a given distance at a given speed
 *
 * @param[in]   distance        Distance in centimeters
 * @param[in]   speed           Move speed, -100 to 100
 */
void db_move_straight(uint16_t distance, int8_t speed);

/**
 * @brief   Rotate by a given angle at a given speed
 *
 * @param[in]   angle           Angle of clockwise rotation in degrees
 * @param[in]   speed           Rotation speed, -100 to 100
 */
void db_move_rotate(uint16_t angle, int8_t speed);

#endif
