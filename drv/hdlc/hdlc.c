/**
 * @file
 * @ingroup drv_hdlc
 *
 * @brief  nRF52833-specific definition of the "hdlc" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdbool.h>
#include <stdint.h>
#include "hdlc.h"

//=========================== definitions ======================================

#define DB_HDLC_BUFFER_SIZE    (UINT8_MAX)  ///< Maximum size of the RX buffer
#define DB_HDLC_FLAG           (0x7E)       ///< Start/End flag
#define DB_HDLC_FLAG_ESCAPED   (0x5E)       ///< Start/End flag escaped
#define DB_HDLC_ESCAPE         (0x7D)       ///< Data escape byte
#define DB_HDLC_ESCAPE_ESCAPED (0x5D)       ///< Escape flag escaped
#define DB_HDLC_FCS_INIT       (0xFFFF)     ///< Initialization value of the FCS
#define DB_HDLC_FCS_OK         (0xF0B8)     ///< Expected value of the FCS

typedef struct {
    uint8_t         buffer[DB_HDLC_BUFFER_SIZE];  ///< Input buffer
    uint8_t         buffer_pos;                   ///< Current position in the input buffer
    db_hdlc_state_t state;                        ///< Current state of the HDLC RX engine
    uint16_t        fcs;                          ///< Current value of the FCS
} hdlc_vars_t;

//=========================== variables ========================================
// clang-format off
static const uint16_t _fcs[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};
// clang-format on

static hdlc_vars_t _hdlc_vars;

//=========================== prototypes =======================================

uint16_t _db_hdlc_update_fcs(uint16_t fcs, uint8_t byte);

//=========================== public ===========================================

db_hdlc_state_t db_hdlc_rx_byte(uint8_t byte) {
    const bool can_handle_new_frame = (_hdlc_vars.state == DB_HDLC_STATE_IDLE ||
                                       _hdlc_vars.state == DB_HDLC_STATE_ERROR ||
                                       _hdlc_vars.state == DB_HDLC_STATE_READY);
    if (can_handle_new_frame && byte == DB_HDLC_FLAG) {
        // Beginning of frame
        _hdlc_vars.buffer_pos = 0;
        _hdlc_vars.fcs        = DB_HDLC_FCS_INIT;
        _hdlc_vars.state      = DB_HDLC_STATE_RECEIVING;
    } else if (_hdlc_vars.buffer_pos > 0 && _hdlc_vars.state == DB_HDLC_STATE_RECEIVING && byte == DB_HDLC_FLAG) {
        // End of frame
        if (_hdlc_vars.fcs != DB_HDLC_FCS_OK) {
            // Invalid FCS
            _hdlc_vars.state = DB_HDLC_STATE_ERROR;
        }
        _hdlc_vars.state = DB_HDLC_STATE_READY;
    } else if (_hdlc_vars.state == DB_HDLC_STATE_RECEIVING) {
        // Middle of frame
        if (_hdlc_vars.buffer_pos >= DB_HDLC_BUFFER_SIZE - 1) {
            // Buffer is full and no end flag was received so something is wrong
            _hdlc_vars.state = DB_HDLC_STATE_ERROR;
            return _hdlc_vars.state;
        }
        _hdlc_vars.buffer[_hdlc_vars.buffer_pos++] = byte;
        _hdlc_vars.fcs                             = _db_hdlc_update_fcs(_hdlc_vars.fcs, byte);
    }

    return _hdlc_vars.state;
}

size_t db_hdlc_decode(uint8_t *output) {
    size_t output_pos = 0;
    if (_hdlc_vars.state != DB_HDLC_STATE_READY) {
        return output_pos;
    }

    uint8_t input_pos   = 0;
    bool    escape_byte = false;
    while (input_pos < _hdlc_vars.buffer_pos) {
        uint8_t current_byte = _hdlc_vars.buffer[input_pos];
        if (current_byte == DB_HDLC_ESCAPE) {
            escape_byte = true;
        } else if (escape_byte == true) {
            if (current_byte == DB_HDLC_ESCAPE_ESCAPED) {
                output[output_pos++] = DB_HDLC_ESCAPE;
            } else if (current_byte == DB_HDLC_FLAG_ESCAPED) {
                output[output_pos++] = DB_HDLC_FLAG;
            }
            escape_byte = false;
        } else {
            output[output_pos++] = _hdlc_vars.buffer[input_pos];
        }
        input_pos++;
    }

    _hdlc_vars.state = DB_HDLC_STATE_IDLE;
    return output_pos - 2;
}

size_t db_hdlc_encode(const uint8_t *input, size_t input_len, uint8_t *frame) {
    uint16_t fcs       = DB_HDLC_FCS_INIT;
    size_t   frame_len = 0;

    // Start flag
    frame[frame_len++] = DB_HDLC_FLAG;

    for (uint8_t pos = 0; pos < input_len; pos++) {
        uint8_t byte = input[pos];
        fcs          = _db_hdlc_update_fcs(fcs, byte);
        if (byte == DB_HDLC_ESCAPE) {
            frame[frame_len++] = DB_HDLC_ESCAPE;
            frame[frame_len++] = DB_HDLC_ESCAPE_ESCAPED;
        } else if (byte == DB_HDLC_FLAG) {
            frame[frame_len++] = DB_HDLC_ESCAPE;
            frame[frame_len++] = DB_HDLC_FLAG_ESCAPED;
        } else {
            frame[frame_len++] = byte;
        }
    }

    fcs = 0xFFFF - fcs;

    // Write the FCS in the frame
    if ((fcs & 0xFF) == DB_HDLC_ESCAPE) {
        frame[frame_len++] = DB_HDLC_ESCAPE;
        frame[frame_len++] = DB_HDLC_ESCAPE_ESCAPED;
    } else if ((fcs & 0xFF) == DB_HDLC_FLAG) {
        frame[frame_len++] = DB_HDLC_ESCAPE;
        frame[frame_len++] = DB_HDLC_FLAG_ESCAPED;
    } else {
        frame[frame_len++] = (fcs & 0xFF);
    }
    if (((fcs & 0xFF00) >> 8) == DB_HDLC_ESCAPE) {
        frame[frame_len++] = DB_HDLC_ESCAPE;
        frame[frame_len++] = DB_HDLC_ESCAPE_ESCAPED;
    } else if (((fcs & 0xFF00) >> 8) == DB_HDLC_FLAG) {
        frame[frame_len++] = DB_HDLC_ESCAPE;
        frame[frame_len++] = DB_HDLC_FLAG_ESCAPED;
    } else {
        frame[frame_len++] = ((fcs & 0xFF00) >> 8);
    }

    // End flag
    frame[frame_len++] = DB_HDLC_FLAG;

    return frame_len;
}

//=========================== private ==========================================

uint16_t _db_hdlc_update_fcs(uint16_t fcs, uint8_t byte) {
    return (fcs >> 8) ^ _fcs[(fcs ^ byte) & 0xff];
}
