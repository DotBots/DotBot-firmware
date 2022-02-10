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

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>


//=========================== public ======================================

void db_radio_init(void (*callback)());

void db_radio_set_frequency(uint8_t freq);

void db_radio_tx_send(uint8_t *package, uint8_t length);

void db_radio_rx_enable(void);
void db_radio_rx_disable(void);

#endif
