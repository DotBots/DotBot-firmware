/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is an example on how to use the TDMA server
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
#include "tdma_server.h"

//=========================== defines ==========================================

#define DELAY_MS   (1000)                ///< Wait between each send
#define RADIO_FREQ (28)                  ///< Set the frequency to 2412 MHz
#define RADIO_MODE (DB_RADIO_BLE_1MBit)  ///< Use BLE 1Mbit/s

//=========================== variables ========================================

static uint8_t            packet_tx[300] = { 0 };
static tdma_table_entry_t clients[12]    = { 0 };
static uint32_t           frame_duration_us;
static uint16_t           num_clients;
static uint16_t           table_index;

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

    // Initialize the TDMA server
    db_tdma_server_init(&radio_callback, RADIO_MODE, RADIO_FREQ);

    while (1) {
        db_tdma_server_get_table_info(&frame_duration_us, &num_clients, &table_index);
        for (size_t i = 0; i < 12; i++) {
            db_tdma_server_get_client_info(&clients[i], i);
        }
        // Print current status
        printf("[*] Frame duration = {%d}\n", frame_duration_us);
        printf("[*] Num. of Clients = {%d}\n", num_clients);
        printf("[*] Server   = {%x}\n", (uint16_t)(clients[0].client >> 48));
        printf("[*] Client 1 = {%x}\n", (uint16_t)(clients[1].client >> 48));
        printf("[*] Client 2 = {%x}\n", (uint16_t)(clients[2].client >> 48));
        printf("[*] Client 3 = {%x}\n", (uint16_t)(clients[3].client >> 48));
        printf("[*] Client 4 = {%x}\n", (uint16_t)(clients[4].client >> 48));
        printf("[*] Client 5 = {%x}\n", (uint16_t)(clients[5].client >> 48));
        printf("[*] Client 6 = {%x}\n", (uint16_t)(clients[6].client >> 48));
        printf("[*] Client 7 = {%x}\n", (uint16_t)(clients[7].client >> 48));
        printf("[*] Client 8 = {%x}\n", (uint16_t)(clients[8].client >> 48));
        printf("[*] Client 9 = {%x}\n", (uint16_t)(clients[9].client >> 48));
        printf("[*] Client 10 = {%x}\n", (uint16_t)(clients[10].client >> 48));

        // Send an advertisement message
        db_protocol_advertizement_to_buffer(packet_tx, DB_BROADCAST_ADDRESS, DotBot, false);
        size_t length = sizeof(protocol_header_t);
        db_tdma_server_tx((uint8_t *)packet_tx, length);

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
