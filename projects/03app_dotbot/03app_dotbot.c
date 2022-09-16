/**
 * @file 03app_dotbot.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control a single dotbot remotely
 *
 * The remote control can be either a keyboard, a joystick or buttons on the gateway
 * itself
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdlib.h>
// Include BSP headers
#include "board.h"
#include "device.h"
#include "protocol.h"
#include "motors.h"
#include "radio.h"
#include "rgbled.h"
#include "timer_hf.h"

//=========================== defines ==========================================

#define TIMEOUT_CHECK_DELAY_US (100 * 1000)  ///< 100 ms delay between packet received timeout checks

typedef struct {
    uint32_t ts_last_packet_received;  ///< Last timestamp in microseconds a control packet was received
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;

//=========================== prototypes =======================================

static void _timeout_check(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    _dotbot_vars.ts_last_packet_received = db_timer_hf_now();
    do {
        uint8_t *          ptk_ptr = pkt;
        protocol_header_t *header  = (protocol_header_t *)ptk_ptr;
        // Check version is supported
        if (header->version != DB_PROTOCOL_VERSION) {
            break;
        }

        // Check destination address matches
        if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
            break;
        }

        uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
        // parse received packet and update the motors' speeds
        switch (header->type) {
            case DB_PROTOCOL_CMD_MOVE_RAW:
            {
                protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
                int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
                int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
                db_motors_set_speed(left, right);
            } break;
            case DB_PROTOCOL_CMD_RGB_LED:
            {
                protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
                db_rgbled_set(command->r, command->g, command->b);
            } break;
        }
    } while (0);
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_rgbled_init();
    db_motors_init();
    db_radio_init(&radio_callback);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();       // Start receiving packets.
    db_timer_hf_init();
    db_timer_hf_set_periodic_us(0, TIMEOUT_CHECK_DELAY_US, &_timeout_check);

    while (1) {
        __WFE();  // Enter a low power state while waiting.
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== private functions ================================

static void _timeout_check(void) {
    uint32_t now = db_timer_hf_now();
    if (now > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_US) {
        db_motors_set_speed(0, 0);
    }
}
