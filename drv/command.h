#ifndef __COMMAND_H
#define __COMMAND_H

/**
 * @file command.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "command" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

#define DB_COMMAND_VERSION      (0)     ///< Version of the command protocol

typedef enum {
    DB_COMMAND_MOVE_RAW = 0,            ///< Move raw command type
    DB_COMMAND_RGB_LED  = 1,            ///< RGB LED command type
} command_type_t;

typedef struct __attribute__((packed)) {
    int8_t left_x;                      ///< Horizontal coordinate for left side
    int8_t left_y;                      ///< Vertical coordinate for left side
    int8_t right_x;                     ///< Horizontal coordinate for right side
    int8_t right_y;                     ///< Vertical coordinate for right side
} move_raw_command_t;

typedef struct __attribute__((packed)) {
    uint8_t r;                          ///< Red component value
    uint8_t g;                          ///< Green component value
    uint8_t b;                          ///< Blue component value
} rgbled_command_t;

//=========================== public ===========================================

void db_command_move_raw_to_buffer(uint8_t *buffer, move_raw_command_t *command);
void db_command_rgbled_to_buffer(uint8_t *buffer, rgbled_command_t *command);

#endif
