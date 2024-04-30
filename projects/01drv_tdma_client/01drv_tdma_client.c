/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is an example on how to use the TDMA client
 *
 * @copyright Inria, 2024
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "board_config.h"
#include "board.h"
#include "timer.h"
#include "protocol.h"
#include "tdma_client.h"

//=========================== defines ==========================================

#define DELAY_MS   (300)                 // Wait 100ms between each send
#define RADIO_FREQ (12)                  // Set the frequency to 2412 MHz
#define RADIO_MODE (DB_RADIO_BLE_1MBit)  // Use BLE 1Mbit/s

//=========================== variables ========================================

static uint8_t      packet_tx[300] = { 0 };
tdma_client_table_t tdma_table     = { 0 };

//========================== prototypes ========================================

static void radio_callback(uint8_t *packet, uint8_t length);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();
    db_timer_init();
        NRF_P0  ->DIRSET = 1<<26;
    NRF_P1  ->DIRSET = 1<<13;
    NRF_P1->DIRSET = 1<<10;

    // Initialize the TDMA client
    db_tdma_client_init(&radio_callback, RADIO_MODE, RADIO_FREQ);

    while (1) {
        // Print current status
        if (db_tdma_client_get_status() == DB_TDMA_CLIENT_UNREGISTERED) {
            printf("Status: UNREGISTERED. Sending packet\n");
        } else {
            db_tdma_client_get_table(&tdma_table);
            printf("Status: REGISTERED. Sending packet\n");
            printf("[*] Frame duration = {%d}\n", tdma_table.frame_duration);
            printf("[*] TX start = {%d}\n", tdma_table.tx_start);
            printf("[*] TX Duration = {%d}\n", tdma_table.tx_duration);
        }

        // Send an advertisement message
        db_protocol_header_to_buffer(packet_tx, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
        size_t length = sizeof(protocol_header_t);
        db_tdma_client_tx((uint8_t *)packet_tx, length);

        // Wait a bit before sending another message
        db_timer_delay_ms(DELAY_MS);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== Callbacks ===============================

static void radio_callback(uint8_t *packet, uint8_t length) {
    printf("packet received (%dB): %s\n", length, (char *)packet);
}