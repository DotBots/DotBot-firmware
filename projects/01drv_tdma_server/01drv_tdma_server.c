/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is an example on how to use the TDMA server
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
#include "timer_hf.h"
#include "protocol.h"
#include "tdma_server.h"

//=========================== defines ==========================================

#define DELAY_MS   (500)                 // Wait 100ms between each send
#define RADIO_FREQ (12)                  // Set the frequency to 2412 MHz
#define RADIO_MODE (DB_RADIO_BLE_1MBit)  // Use BLE 1Mbit/s

//=========================== variables ========================================

static uint8_t       packet_tx[300] = { 0 };
tdma_server_table_t *tdma_table_ptr;

//========================== prototypes ========================================

static void radio_callback(uint8_t *packet, uint8_t length);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the TDMA server
    db_tdma_server_init(&radio_callback, RADIO_MODE, RADIO_FREQ);

    while (1) {
        // Print current status
        tdma_table_ptr = db_tdma_server_get_table();
        printf("[*] Frame duration = {%d}\n", tdma_table_ptr->frame_duration_us);
        printf("[*] Num. of Clients = {%d}\n", tdma_table_ptr->num_clients);
        printf("[*] Client 1 = {%x}\n", tdma_table_ptr->table[0]);
        printf("[*] Client 2 = {%x}\n", tdma_table_ptr->table[1]);
        printf("[*] Client 3 = {%x}\n", tdma_table_ptr->table[2]);

        // Send an advertisement message
        db_protocol_header_to_buffer(packet_tx, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
        size_t length = sizeof(protocol_header_t);
        db_tdma_server_tx((uint8_t *)packet_tx, length);

        // Wait a bit before sending another message
        db_timer_hf_delay_ms(DELAY_MS);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== Callbacks ===============================

static void radio_callback(uint8_t *packet, uint8_t length) {
    printf("packet received (%dB): %s\n", length, (char *)packet);
}