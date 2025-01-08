/**
 * @file
 * @defgroup project_xgo    XGO application
 * @ingroup projects
 * @brief This application is used to control an XGO robot remotely
 *
 * The serial communication protocol is documented at http://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/serial-protocol
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 *
 */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <nrf.h>
// Include BSP headers
#include "device.h"
#include "protocol.h"
#include "radio.h"
#include "timer_hf.h"
#include "uart.h"
// Include DRV headers
#include "tdma_client.h"

//=========================== defines ==========================================

#define XGO_ADVERTIZEMENT_DELAY_US (1000 * 500UL)  ///< 500ms delay between each advertizement packet sending
#define XGO_TIMEOUT_CHECK_DELAY_US (1000 * 200UL)  ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_US     (1000 * 500UL)  ///< 500 ms delay between packet received timeout checks

#define XGO_RADIO_BUFFER_MAX_BYTES (255U)
#define XGO_RADIO_FREQ             (8)  //< Set the frequency to 2408 MHz
#define XGO_UART_BAUDRATE          (115200U)
#define XGO_UART_MAX_BYTES         (64U)

#define XGO_FRAME_HEADER1 (0x55)
#define XGO_FRAME_HEADER2 (0x00)
#define XGO_FRAME_END1    (0x00)
#define XGO_FRAME_END2    (0xAA)

#define XGO_COMMAND_WRITE (0x00)
#define XGO_COMMAND_READ  (0x02)

#define XGO_ADDRESS_STATUS               (0x00)
#define XGO_ADDRESS_BATT_LEVEL           (0x01)
#define XGO_ADDRESS_SHOW_MODE            (0x03)
#define XGO_ADDRESS_CALIBRATION_MODE     (0x04)
#define XGO_ADDRESS_UPDATE_FIRMWARE      (0x05)
#define XGO_ADDRESS_FIRMWARE_VERSION     (0x07)
#define XGO_ADDRESS_GAIT                 (0x09)
#define XGO_ADDRESS_UNINSTALL_SERVO      (0x20)
#define XGO_ADDRESS_RESET_SERVO_ZERO_POS (0x21)
#define XGO_ADDRESS_MOVE_X_SPEED         (0x30)
#define XGO_ADDRESS_MOVE_Y_SPEED         (0x31)
#define XGO_ADDRESS_ROTATE_SPEED         (0x32)
#define XGO_ADDRESS_ACTION_COMMAND       (0x3E)

typedef enum {
    XGO_ACTION_GET_DOWN            = 1,
    XGO_ACTION_STAND_UP            = 2,
    XGO_ACTION_CREEP_FORWARD       = 3,
    XGO_ACTION_CIRCLE_AROUND       = 4,
    XGO_ACTION_SQUAT_UP            = 6,
    XGO_ACTION_TURN_ROLL           = 7,
    XGO_ACTION_TURN_PITCH          = 8,
    XGO_ACTION_TURN_YAW            = 9,
    XGO_ACTION_THREE_AXIS_ROTATION = 10,
    XGO_ACTION_PEE                 = 11,
    XGO_ACTION_SIT_DOWN            = 12,
    XGO_ACTION_WAVE                = 13,
    XGO_ACTION_STRETCH             = 14,
    XGO_ACTION_WAVE2               = 15,
    XGO_ACTION_SWING_LEFT_RIGHT    = 16,
    XGO_ACTION_BEGGING_FOOD        = 17,
    XGO_ACTION_LOOKING_FOOD        = 18,
    XGO_ACTION_SHAKE_HANDS         = 19,
    XGO_ACTION_CHICKEN_HEAD        = 20,
    XGO_ACTION_PUSH_UPS            = 21,
    XGO_ACTION_LOOK_AROUND         = 22,
    XGO_ACTION_DANCE               = 23,
    XGO_ACTION_NAUGHTY             = 24,
    XGO_ACTION_CATCH_UP            = 128,
    XGO_ACTION_CAUGHT              = 129,
    XGO_ACTION_CATCH               = 130,
    XGO_ACTION_RESTORE             = 255,
} xgo_action_id_t;

typedef struct __attribute__((packed)) {
    uint8_t header1;
    uint8_t header2;
    uint8_t length;
    uint8_t command_type;
    uint8_t address;
    uint8_t data;
    uint8_t crc;
    uint8_t end_frame1;
    uint8_t end_frame2;
} xgo_write_frame_t;

typedef struct {
    uint32_t ts_last_packet_received;
    uint8_t  radio_buffer[XGO_RADIO_BUFFER_MAX_BYTES];
    bool     packet_received;
    bool     advertize;
    bool     timeout_check;
    uint64_t device_id;
    bool     action_running;
} xgo_vars_t;

//=========================== variables ========================================

#if defined(BOARD_XGO_V1)
static const gpio_t _uart_rx = { .port = 0, .pin = 3 };
static const gpio_t _uart_tx = { .port = 0, .pin = 4 };
#elif defined(BOARD_XGO_V2)
static const gpio_t _uart_rx = { .port = 0, .pin = 17 };
static const gpio_t _uart_tx = { .port = 0, .pin = 1 };
#else
static const gpio_t _uart_rx = { .port = 0, .pin = 13 };  // RX connected to P0.13 on nrf52 DKs
static const gpio_t _uart_tx = { .port = 0, .pin = 14 };  // TX connected to P0.14 on nrf52 DKs
#endif

static const uint8_t _action_execution_time_s[] = {
    [XGO_ACTION_GET_DOWN]            = 3,
    [XGO_ACTION_STAND_UP]            = 3,
    [XGO_ACTION_CREEP_FORWARD]       = 5,
    [XGO_ACTION_CIRCLE_AROUND]       = 5,
    [XGO_ACTION_SQUAT_UP]            = 4,
    [XGO_ACTION_TURN_ROLL]           = 4,
    [XGO_ACTION_TURN_PITCH]          = 4,
    [XGO_ACTION_TURN_YAW]            = 4,
    [XGO_ACTION_THREE_AXIS_ROTATION] = 7,
    [XGO_ACTION_PEE]                 = 7,
    [XGO_ACTION_SIT_DOWN]            = 5,
    [XGO_ACTION_WAVE]                = 7,
    [XGO_ACTION_STRETCH]             = 10,
    [XGO_ACTION_WAVE2]               = 6,
    [XGO_ACTION_SWING_LEFT_RIGHT]    = 6,
    [XGO_ACTION_BEGGING_FOOD]        = 4,
    [XGO_ACTION_LOOKING_FOOD]        = 6,
    [XGO_ACTION_SHAKE_HANDS]         = 10,
    [XGO_ACTION_CHICKEN_HEAD]        = 9,
    [XGO_ACTION_PUSH_UPS]            = 8,
    [XGO_ACTION_LOOK_AROUND]         = 7,
    [XGO_ACTION_DANCE]               = 6,
    [XGO_ACTION_NAUGHTY]             = 7,
    [XGO_ACTION_CATCH_UP]            = 10,
    [XGO_ACTION_CAUGHT]              = 10,
    [XGO_ACTION_CATCH]               = 10,
    [XGO_ACTION_RESTORE]             = 1,
};

static xgo_vars_t _xgo_vars;

//=========================== private ==========================================

static void _compute_crc(xgo_write_frame_t *frame) {
    uint16_t tmp = frame->length;
    tmp += frame->command_type;
    tmp += frame->address;
    tmp += frame->data;
    frame->crc = 0xff - (uint8_t)(tmp & 0x00ff);
}

static void _write(uint8_t address, uint8_t data) {
    xgo_write_frame_t frame = { 0 };
    frame.header1           = XGO_FRAME_HEADER1;
    frame.header2           = XGO_FRAME_HEADER2;
    frame.end_frame1        = XGO_FRAME_END1;
    frame.end_frame2        = XGO_FRAME_END2;
    frame.length            = 0x09;
    frame.command_type      = XGO_COMMAND_WRITE;
    frame.address           = address;
    frame.data              = data;
    _compute_crc(&frame);
    db_uart_write(0, (uint8_t *)&frame, sizeof(xgo_write_frame_t));
}

static void _action(xgo_action_id_t action_id) {
    _xgo_vars.action_running = true;
    _write(XGO_ADDRESS_ACTION_COMMAND, action_id);
    // Wait for the action to complete
    db_timer_hf_delay_s(0, _action_execution_time_s[action_id]);
    _xgo_vars.action_running = false;
}

//=========================== callbacks ========================================

static void _timeout_check(void) {
    if (_xgo_vars.timeout_check) {
        return;
    }
    _xgo_vars.timeout_check = true;
}

static void _advertize(void) {
    if (_xgo_vars.advertize) {
        return;
    }
    _xgo_vars.advertize = true;
}

static void _radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _xgo_vars.packet_received = true;

    uint8_t           *ptk_ptr = pkt;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;
    // Check destination address is matching
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _xgo_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Check application is compatible
    if (header->application != XGO) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and execute the command
    switch (header->type) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            if (_xgo_vars.action_running) {
                // An action is already running, just ignore the command
                return;
            }
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            int16_t                      left    = (int16_t)command->left_y;
            int16_t                      right   = (int16_t)command->right_y;
            if (left != 0 && left == right) {
                // Move forward or backward
                _write(XGO_ADDRESS_MOVE_X_SPEED, (uint8_t)(left + 128));
            } else if (left > 0 && right == 0) {
                // Move to the left
                _write(XGO_ADDRESS_MOVE_Y_SPEED, (uint8_t)((left * -1) + 128));
            } else if (right > 0 && left == 0) {
                // Move to the right
                _write(XGO_ADDRESS_MOVE_Y_SPEED, (uint8_t)(right + 128));
            } else if (left > 0 && right > left) {
                // Turn left
                _write(XGO_ADDRESS_ROTATE_SPEED, (uint8_t)(left + 128));
            } else if (right > 0 && left > right) {
                // Turn right
                _write(XGO_ADDRESS_ROTATE_SPEED, (uint8_t)((right * -1) + 128));
            } else {
                // Stop
                _action(XGO_ACTION_RESTORE);
            }
        } break;
        case DB_PROTOCOL_CMD_XGO_ACTION:
            if (_xgo_vars.action_running) {
                // An action is already running, just ignore the command
                return;
            }
            _action((uint8_t)*cmd_ptr);
            break;
        default:
            break;
    }
}

//=========================== main =============================================

int main(void) {
    db_protocol_init();
    db_tdma_client_init(&_radio_callback, DB_RADIO_BLE_1MBit, XGO_RADIO_FREQ, XGO);

    // Retrieve the device id once at startup
    _xgo_vars.device_id = db_device_id();

    db_uart_init(0, &_uart_rx, &_uart_tx, XGO_UART_BAUDRATE, NULL);

    db_timer_hf_init(0);
    db_timer_hf_set_periodic_us(0, 0, XGO_TIMEOUT_CHECK_DELAY_US, &_timeout_check);
    db_timer_hf_set_periodic_us(0, 1, XGO_ADVERTIZEMENT_DELAY_US, &_advertize);

    while (1) {
        __WFE();
        if (_xgo_vars.packet_received) {
            _xgo_vars.ts_last_packet_received = db_timer_hf_now(0);
            _xgo_vars.packet_received         = false;
        }

        if (_xgo_vars.advertize) {
            db_protocol_header_to_buffer(_xgo_vars.radio_buffer, DB_BROADCAST_ADDRESS, XGO, DB_PROTOCOL_ADVERTISEMENT);
            db_tdma_client_tx(_xgo_vars.radio_buffer, sizeof(protocol_header_t));
            _xgo_vars.advertize = false;
        }

        if (_xgo_vars.timeout_check) {
            uint32_t now = db_timer_hf_now(0);
            if (now > _xgo_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_US) {
                _action(XGO_ACTION_RESTORE);
            }
            _xgo_vars.timeout_check = false;
        }
    }
}
