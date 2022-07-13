/**
 * @file command.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "command" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <string.h>
#include "command.h"

//=========================== public ===========================================

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the move raw command
 */
void db_command_move_raw_to_buffer(uint8_t *buffer, move_raw_command_t *command) {
    buffer[0] = DB_COMMAND_VERSION;
    buffer[1] = DB_COMMAND_MOVE_RAW;
    memcpy(&buffer[2], command, sizeof(move_raw_command_t));
}

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the rgbled command
 */
void db_command_rgbled_to_buffer(uint8_t *buffer, rgbled_command_t *command) {
    buffer[0] = DB_COMMAND_VERSION;
    buffer[1] = DB_COMMAND_RGB_LED;
    memcpy(&buffer[2], command, sizeof(rgbled_command_t));
}
