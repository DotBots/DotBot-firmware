/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is an example on how to use the TDMA client
 *
 * @copyright Inria, 2024
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

#include "board_config.h"
#include "board.h"
#include "timer.h"
#include "protocol.h"
#include "tdma_client.h"

//=========================== defines ==========================================

#define DELAY_MS   (500)                 // Wait 500ms between each send
#define RADIO_FREQ (28)                  // Set the frequency to 2412 MHz
#define RADIO_MODE (DB_RADIO_BLE_1MBit)  // Use BLE 1Mbit/s

//=========================== variables ========================================

static uint8_t             packet_tx[300] = { 0 };
static tdma_client_table_t tdma_table     = { 0 };

//========================== prototypes ========================================

static void radio_callback(uint8_t *packet, uint8_t length);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();
    db_timer_init(0);

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
        db_protocol_advertizement_to_buffer(packet_tx, DB_GATEWAY_ADDRESS, DotBot, false);
        size_t length = sizeof(protocol_header_t);
        // Send 7 messages in a row, to test the QUEUE system
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);
        db_tdma_client_tx((uint8_t *)packet_tx, length);

        // Wait a bit before sending another message
        db_timer_delay_ms(0, DELAY_MS);
    }
}

//=========================== Callbacks ===============================

static void radio_callback(uint8_t *packet, uint8_t length) {
    if (packet[0] == length) {
        __NOP();
    }
}
