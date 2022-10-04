/**
 * @file 03app_sailbot.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is the radio-controlled SailBot app.
 *
 * Load this program on your board. Now the SailBot can be remotely controlled
 * from a nearby nRF52840-DK.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

// Include BSP packages
#include "device.h"
#include "radio.h"
#include "servos.h"
#include "protocol.h"

//=========================== defines =========================================

#define NUM_COMMANDS_FIFO 20

typedef enum command {
    COMMAND_NONE   = 0,
    COMMAND_RUDDER = 1,
    COMMAND_SAILS  = 2,
} sailbot_command_type_t;

typedef struct {
    sailbot_command_type_t command;
    int8_t                 angle;
} sailbot_command_t;

typedef struct queue {
    sailbot_command_t commands[NUM_COMMANDS_FIFO];
    int               head;
    int               tail;
} queue_t;

//=========================== variables =========================================

queue_t queue;

//=========================== prototypes =========================================

void fifo_init();
int  fifo_write(sailbot_command_t val);
void fifo_read(sailbot_command_t *command);
void radio_callback(uint8_t *packet, uint8_t length);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    sailbot_command_t received_command;
    uint32_t          command;
    uint16_t          duration;

    received_command.command = COMMAND_NONE;

    // Configure Radio as a receiver
    db_radio_init(&radio_callback);  // Set the callback function.
    db_radio_set_frequency(8);       // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();            // Start receiving packets.

    // Configure Motors
    servos_init();
    // Wait for radio packets to arrive/
    while (1) {
        // prepare to execute a burst of commands

        // processor idle until an interrupt occurs and is handled
        __WFE();

        do {
            // check if there is something to consume
            fifo_read(&received_command);
            switch (received_command.command) {
                case COMMAND_RUDDER:
                    servos_rudder_turn(received_command.angle);
                    break;
                case COMMAND_SAILS:
                    servos_sail_turn(received_command.angle);
                    break;
                default:
                    break;
            }
        } while (1);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions =========================================

/**
 *  @brief Callback function to process received packets
 *
 * This function gets called each time a packet is received.
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void radio_callback(uint8_t *packet, uint8_t length) {

    uint8_t            type;
    int8_t             left_x;
    int8_t             left_y;
    int8_t             right_x;
    int8_t             right_y;
    static int8_t      right_y_last_position = 0;
    sailbot_command_t  new_command;
    uint8_t *          ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // we filter out all packets other than MOVE_RAW command of length 6 in total
    // FIXME packet sent by BotController is 32 bytes in length
    if (length != 32) {
        return;
    }

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
        return;
    }

    // Check version is compatible
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Only move raw commands are supported
    if (header->type != DB_PROTOCOL_CMD_MOVE_RAW) {
        return;
    }

    // Read the DotBot command
    protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)(ptk_ptr + sizeof(protocol_header_t));

    if (right_y_last_position + command->right_y >= 127) {
        right_y_last_position = 127;
    } else if (right_y_last_position + command->right_y <= -127) {
        right_y_last_position = -127;
    } else {
        right_y_last_position += command->right_y / 3;
    }

    // add the commands to the fifo
    new_command.command = COMMAND_RUDDER;
    new_command.angle   = command->left_x;
    fifo_write(new_command);

    new_command.command = COMMAND_SAILS;
    new_command.angle   = right_y_last_position;
    fifo_write(new_command);
}

void fifo_read(sailbot_command_t *ret) {
    ret->command = COMMAND_NONE;
    if (queue.tail == queue.head) {
        return;
    }
    queue.head = (queue.head + 1) % NUM_COMMANDS_FIFO;
    *ret       = queue.commands[queue.head];
}

int fifo_write(sailbot_command_t val) {
    if (queue.tail - queue.head == NUM_COMMANDS_FIFO - 1) {
        return -1;
    }
    queue.tail                 = (queue.tail + 1) % NUM_COMMANDS_FIFO;
    queue.commands[queue.tail] = val;
    return 0;
}

void fifo_init() {
    queue.tail = 0;
    queue.head = 0;
}
