/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control the radio and interact with the application core
 *
 * @copyright Inria, 2023
 *
 */
#include <stdio.h>
#include <nrf.h>
// Include BSP headers
#include "radio.h"

//=========================== defines ===========================================

#define NUMBER_OF_BYTES_IN_PACKET 32

//=========================== variables =========================================

// static uint8_t packet_tx[NUMBER_OF_BYTES_IN_PACKET];

//=========================== functions =========================================

void radio_callback(uint8_t *packet, uint8_t length) {
    (void)length;

    printf("Packet received (%dB): %s\n", length, (char *)packet);
}

//=========================== main ==============================================

int main(void) {

    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.

    // db_radio_tx(packet_tx, NUMBER_OF_BYTES_IN_PACKET);
    db_radio_rx_enable();  // Start receiving packets.

    while (1) {
        __WFE();
    };
}
