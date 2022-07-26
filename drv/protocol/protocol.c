/**
 * @file command.c
 * @addtogroup DRV
 *
 * @brief  nRF52833-specific definition of the "protocol" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>
#include <string.h>
#include "protocol.h"

//=========================== public ===========================================

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the move raw command
 */
void db_protocol_cmd_move_raw_to_buffer(uint8_t *buffer, protocol_move_raw_command_ht *command) {
    uint8_t *hdr_ptr = buffer;
    uint8_t *cmd_ptr = buffer + sizeof(protocol_header_ht);
    protocol_header_ht header = { .version = DB_PROTOCOL_VERSION, .type = DB_PROTOCOL_CMD_MOVE_RAW };
    memcpy(hdr_ptr, &header, sizeof(protocol_header_ht));
    memcpy(cmd_ptr, command, sizeof(protocol_move_raw_command_ht));
}

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   command     Pointer to the rgbled command
 */
void db_protocol_cmd_rgbled_to_buffer(uint8_t *buffer, protocol_rgbled_command_ht *command) {
    uint8_t *hdr_ptr = buffer;
    uint8_t *cmd_ptr = buffer + sizeof(protocol_header_ht);
    protocol_header_ht header = { .version = DB_PROTOCOL_VERSION, .type = DB_PROTOCOL_CMD_RGB_LED };
    memcpy(hdr_ptr, &header, sizeof(protocol_header_ht));
    memcpy(cmd_ptr, command, sizeof(protocol_rgbled_command_ht));
}
