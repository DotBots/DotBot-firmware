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
#include <stdint.h>
#include <nrf.h>

//=========================== defines ==========================================

typedef void (*radio_cb_t)(uint8_t *packet, uint8_t length);    ///< Function pointer to the callback function called on packet receive

//=========================== public ===========================================

void db_radio_init(radio_cb_t callback);

void db_radio_init_lr(radio_cb_t callback);

void db_radio_set_frequency(uint8_t freq);

void db_radio_tx(uint8_t *package, uint8_t length);

void db_radio_rx_enable(void);
void db_radio_rx_disable(void);

#endif
