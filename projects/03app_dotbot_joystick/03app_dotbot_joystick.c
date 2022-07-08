/**
 * @file 03app_dotbot_remote_control.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control a single dotbot remotely with a joystick using radio.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
// Include BSP headers
#include "board.h"
#include "command.h"
#include "motors.h"
#include "radio.h"
#include "rgbled.h"
#include "timer_hf.h"

//=========================== defines =========================================

#define MAX_RECEIVE_DELAY_US    (100 * 1000)

static uint32_t _last_packet_received = 0;

//=========================== main =========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    _last_packet_received = db_timer_hf_now();
    // Check version is supported
    if (pkt[0] != DB_COMMAND_VERSION) {
        return;
    }

    // parse received packet and update the motors' speeds
    switch (pkt[1]) {
        case DB_COMMAND_MOVE_RAW:
        {
            move_raw_command_t *command = (move_raw_command_t *)&pkt[2];
            int16_t left = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t right = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            printf("Move: %i-%i\n", left, right);
            db_motors_set_speed(left, right);
        }
            break;
        case DB_COMMAND_RGB_LED:
        {
            rgbled_command_t *command = (rgbled_command_t *)&pkt[2];
            printf("%d-%d-%d\n", command->r, command->g, command->b
            );
            db_rgbled_set(command->r, command->g, command->b);
        }
            break;
    }
}

static void timeout_check(void) {
    uint32_t now = db_timer_hf_now();
    if (now > _last_packet_received + MAX_RECEIVE_DELAY_US) {
        db_motors_set_speed(0, 0);
        puts("stopping motors");
    }
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    puts("DotBot joystick");
    db_board_init();
    db_rgbled_init();
    db_motors_init();
    db_radio_init(&radio_callback);
    db_radio_set_frequency(8);      // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();           // Start receiving packets.
    db_timer_hf_init();
    db_timer_hf_set_periodic(0, MAX_RECEIVE_DELAY_US, &timeout_check);

    while (1) {
        __WFE(); // Enter a low power state while waiting.
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
