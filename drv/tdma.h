#ifndef __TDMA_H
#define __TDMA_H

/**
 * @defgroup    drv_tdma      TDMA radio driver
 * @ingroup     drv
 * @brief       Driver for Time-Division-Multiple-Access fot the DotBot radio
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2024
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"
#include "radio.h"

//=========================== defines ==========================================

/**
 * @brief   Accelerometer Registers
 * @{
 */
// #define ISM330_REG_OUTX_L_A 0x28  ///< Accelerometer OUTX Low
// #define ISM330_REG_OUTX_H_A 0x29  ///< Accelerometer OUTX High
// #define ISM330_REG_OUTY_L_A 0x2A  ///< Accelerometer OUTY Low
// #define ISM330_REG_OUTY_H_A 0x2B  ///< Accelerometer OUTY High
// #define ISM330_REG_OUTZ_L_A 0x2C  ///< Accelerometer OUTZ Low
// #define ISM330_REG_OUTZ_H_A 0x2D  ///< Accelerometer OUTZ High
/** @} */

//=========================== variables ========================================

/// Data type to store the TDMA table
typedef struct {
    uint16_t frame;        ///< Duration of the entire TDMA frame [microseconds]
    uint16_t rx_start;     ///< Time between the start of the frame and when the gateway starts transmitting
    uint16_t rx_duration;  ///< Duration the gateway will transmit messages
    uint16_t tx_start;     ///< Time between the start of the frame and the start of the DotBot's alloted frame
    uint16_t tx_duration;  ///< Duration of the DotBot's alloted frame.
} tdma_table_t;

typedef void (*tdma_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

//=========================== prototypes ==========================================

/**
 * @brief Initializes the TDMA scheme
 *
 * Starts advertising registration packets, set a default tdma table
 * and innits the radio
 *
 * @param[in] callback      pointer to a function that will be called each time a packet is received.
 * @param[in] radio_mode    BLE mode used by the radio (1MBit, 2MBit, LR125KBit, LR500Kbit)
 * @param[in] freq          Frequency of the radio [0, 100]
 * @param[in] buffer_size   Number of messages that can be queued for sending via radio.
 *
 */
void db_tdma_init(tdma_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq, uint8_t buffer_size);

/**
 * @brief Updates the RX and TX timings for the TDMA table
 *
 * @param[in] table       New table of TDMA timings
 */
void db_tdma_update_table(tdma_table_t *table);

/**
 * @brief Queues a single packet to send through the Radio
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void db_tdma_tx(const uint8_t *packet, uint8_t length);

/**
 * @brief Ignore TDMA table and send all pending packets inmediatly
 *
 */
void db_tdma_flush();

/**
 * @brief Earese all pending packets in the TDMA queue
 *
 */
void db_tdma_empty();

#endif
