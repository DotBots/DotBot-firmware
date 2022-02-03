#ifndef __RADIO_H
#define __RADIO_H

/**
 * @file radio.h
 * @addtogroup BSP
 * 
 * @brief  Cross-platform declaration "radio" bsp module.
 * 
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * 
 * @copyright Inria, 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== prototypes ======================================

//=========================== public ======================================

void db_radio_tx_init(uint8_t freq, uint8_t log_addr, uint8_t *packet);
void db_radio_rx_init(uint8_t freq, uint8_t log_addr, uint8_t *packet);

void db_radio_set_frequency(uint8_t freq);
void db_radio_set_logical_address(uint8_t log_addr);

void db_radio_send_packet(void);

void db_radio_rx_enable(void);
void db_radio_rx_disable(void);


#endif
