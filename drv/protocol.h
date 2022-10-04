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

#define DB_FIRMWARE_VERSION  (1)                   ///< Version of the command protocol
#define DB_BROADCAST_ADDRESS 0xffffffffffffffffUL  ///< Broadcast address
#define DB_GATEWAY_ADDRESS   0x0000000000000000UL  ///< Gateway address

typedef enum {
    DB_PROTOCOL_CMD_MOVE_RAW = 0,  ///< Move raw command type
    DB_PROTOCOL_CMD_RGB_LED  = 1,  ///< RGB LED command type
} command_type_t;

typedef struct __attribute__((packed)) {
    uint64_t       dst;      ///< Destination address of this packet
    uint64_t       src;      ///< Source address of this packet
    uint8_t        version;  ///< Version of the protocol
    command_type_t type;     ///< Type of command following this header
} protocol_header_t;

typedef struct __attribute__((packed)) {
    int8_t left_x;   ///< Horizontal coordinate for left side
    int8_t left_y;   ///< Vertical coordinate for left side
    int8_t right_x;  ///< Horizontal coordinate for right side
    int8_t right_y;  ///< Vertical coordinate for right side
} protocol_move_raw_command_t;

typedef struct __attribute__((packed)) {
    uint8_t r;  ///< Red component value
    uint8_t g;  ///< Green component value
    uint8_t b;  ///< Blue component value
} protocol_rgbled_command_t;

//=========================== public ===========================================

/**
 * @brief   Write the protocol header in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   type        Command type that follows this header
 */
void db_protocol_header_to_buffer(uint8_t *buffer, uint64_t dst, command_type_t type);

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   command     Pointer to the move raw command
 */
void db_protocol_cmd_move_raw_to_buffer(uint8_t *buffer, uint64_t dst, protocol_move_raw_command_t *command);

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   command     Pointer to the rgbled command
 */
void db_protocol_cmd_rgbled_to_buffer(uint8_t *buffer, uint64_t dst, protocol_rgbled_command_t *command);

#endif
