#ifndef __TSCH_H
#define __TSCH_H

/**
 * @defgroup    drv_tsch      TSCH radio driver
 * @ingroup     drv
 * @brief       Driver for Time-Slotted Channel Hopping (TSCH)
 *
 * @{
 * @file
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 * @copyright Inria, 2024-now
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

#include "gpio.h"
#include "radio.h"
#include "protocol.h"

//=========================== defines ==========================================

#define TSCH_TIMER_DEV 2 ///< HF timer device used for the TSCH scheduler
#define TSCH_TIMER_SLOT_CHANNEL 0
#define TSCH_TIMER_INTRA_SLOT_CHANNEL 1

//=========================== variables ========================================

typedef void (*tsch_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

typedef enum {
    TSCH_RADIO_ACTION_TX,
    TSCH_RADIO_ACTION_RX,
    TSCH_RADIO_ACTION_SLEEP,
} tsch_radio_action_t;

typedef struct {
    tsch_radio_action_t radio_action;
    uint8_t frequency;
    uint16_t duration_us;
} tsch_radio_event_t;

//=========================== prototypes ==========================================

/**
 * @brief Initializes the TSCH scheme
 *
 * @param[in] callback             pointer to a function that will be called each time a packet is received.
 * @param[in] radio_mode           BLE mode used by the radio (1MBit, 2MBit, LR125KBit, LR500Kbit)
 * @param[in] default_radio_app    Which application to use for registration and sync messages
 *
 */
void db_tsch_init(tsch_cb_t callback, db_radio_mode_t radio_mode, application_type_t default_radio_app);

#endif
