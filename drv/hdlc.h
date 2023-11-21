#ifndef __HDLC_H
#define __HDLC_H

/**
 * @defgroup    drv_hdlc    HDLC protocol
 * @ingroup     drv
 * @brief       High-Level Data Link Control protocol library
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdlib.h>
#include <stdint.h>

//=========================== definitions ======================================

/// Internal state of the HDLC decoder
typedef enum {
    DB_HDLC_STATE_IDLE,       ///< Waiting for incoming HDLC frames
    DB_HDLC_STATE_RECEIVING,  ///< An HDLC frame is being received
    DB_HDLC_STATE_READY,      ///< An HDLC frame is ready to be decoded
    DB_HDLC_STATE_ERROR,      ///< The FCS value is invalid
} db_hdlc_state_t;

//=========================== public ===========================================

/**
 * @brief   Handle a byte received in HDLC internal state
 *
 * @param[in]   byte    The received byte
 */
db_hdlc_state_t db_hdlc_rx_byte(uint8_t byte);

/**
 * @brief   Decode an HDLC frame
 *
 * @param[out]  payload     Decoded payload contained in the input buffer
 *
 * @return the number of bytes decoded
 */
size_t db_hdlc_decode(uint8_t *payload);

/**
 * @brief   Encode a buffer in an HDLC frame
 *
 * @param[in]   input       Input buffer to encode in the HDLC frame
 * @param[in]   input_len   Number of bytes of the input buffer
 * @param[out]  frame       Buffer containing the output HDLC frame
 *
 * @return the size of the HDLC frame
 */
size_t db_hdlc_encode(const uint8_t *input, size_t input_len, uint8_t *frame);

#endif
