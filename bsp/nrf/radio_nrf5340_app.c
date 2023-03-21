/**
 * @file radio_nrf5340_app.c
 * @addtogroup BSP
 *
 * @brief  nrf5340-app-specific definition of the "radio" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "clock.h"
#include "radio.h"

//=========================== defines ==========================================

#define PAYLOAD_MAX_LENGTH UINT8_MAX

typedef struct __attribute__((packed)) {
    uint8_t header;                       ///< PDU header (depends on the type of PDU - advertising physical channel or Data physical channel)
    uint8_t length;                       ///< Length of the payload + MIC (if any)
    uint8_t payload[PAYLOAD_MAX_LENGTH];  ///< Payload + MIC (if any)
} ble_radio_pdu_t;

typedef struct {
    ble_radio_pdu_t pdu;       ///< Variable that stores the radio PDU (protocol data unit) that arrives and the radio packets that are about to be sent.
    radio_cb_t      callback;  ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
} radio_vars_t;

//=========================== variables ========================================

//========================== prototypes ========================================

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback, db_radio_ble_mode_t mode) {
    (void)callback;
    (void)mode;

    // Start the network core
    NRF_RESET_S->NETWORK.FORCEOFF = 0;
}

void db_radio_set_frequency(uint8_t freq) {
    (void)freq;
}

void db_radio_set_channel(uint8_t channel) {
    (void)channel;
}

void db_radio_set_network_address(uint32_t addr) {
    (void)addr;
}

void db_radio_tx(uint8_t *tx_buffer, uint8_t length) {
    (void)tx_buffer;
    (void)length;
}

void db_radio_rx_enable(void) {
}

void db_radio_rx_disable(void) {
}

//=========================== private ==========================================

//=========================== interrupt handlers ===============================
