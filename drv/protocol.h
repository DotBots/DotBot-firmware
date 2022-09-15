#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/**
 * @file procotol.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "protocol" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

#define DB_PROTOCOL_VERSION (0)  ///< Version of the command protocol

typedef enum {
    DB_PROTOCOL_CMD_MOVE_RAW = 0,  ///< Move raw command type
    DB_PROTOCOL_CMD_RGB_LED  = 1,  ///< RGB LED command type
} command_type_t;

typedef struct __attribute__((packed)) {
    uint8_t        version;  ///< Version of the protocol
    command_type_t type;     ///< Type of command following this header
} protocol_header_ht;

typedef struct __attribute__((packed)) {
    int8_t left_x;   ///< Horizontal coordinate for left side
    int8_t left_y;   ///< Vertical coordinate for left side
    int8_t right_x;  ///< Horizontal coordinate for right side
    int8_t right_y;  ///< Vertical coordinate for right side
} protocol_move_raw_command_ht;

typedef struct __attribute__((packed)) {
    uint8_t r;  ///< Red component value
    uint8_t g;  ///< Green component value
    uint8_t b;  ///< Blue component value
} protocol_rgbled_command_ht;

//=========================== public ===========================================

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the move raw command
 */
void db_protocol_cmd_move_raw_to_buffer(uint8_t *buffer, protocol_move_raw_command_ht *command);

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the rgbled command
 */
void db_protocol_cmd_rgbled_to_buffer(uint8_t *buffer, protocol_rgbled_command_ht *command);

#endif
