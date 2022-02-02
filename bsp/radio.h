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
 * @copyright INRIA, 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== prototypes ======================================

//=========================== public ======================================
void gw_radio_tx_init(uint8_t freq, uint8_t log_addr, uint8_t *packet);
void gw_radio_rx_init(uint8_t freq, uint8_t log_addr, uint8_t *packet);

void gw_radio_set_frequency(uint8_t freq);
void gw_radio_set_logical_address(uint8_t log_addr);

void gw_radio_send_packet(void);

void gw_radio_rx_enable(void);
void gw_radio_rx_disable(void);


#endif