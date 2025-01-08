/**
 * @file
 * @ingroup     drv_tsch
 *
 * @brief       Driver for Time-Slotted Channel Hopping (TSCH)
 *
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "tsch.h"
#include "scheduler.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"
#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "ipc.h"
#endif

#ifndef DEBUG // FIXME: remove before merge. Just to make VS Code enable code behind `#ifdef DEBUG`
#define DEBUG
#endif

#ifdef DEBUG // comes from segger config
#include "gpio.h" // for debugging
gpio_t pin0 = { .port = 1, .pin = 2 }; // variable names reflect the logic analyzer channels
gpio_t pin1 = { .port = 1, .pin = 3 };
gpio_t pin2 = { .port = 1, .pin = 4 };
gpio_t pin3 = { .port = 1, .pin = 5 };
#endif

#ifdef DEBUG
    #define DEBUG_GPIO_TOGGLE(pin) db_gpio_toggle(pin)
    #define DEBUG_GPIO_SET(pin) db_gpio_set(pin)
    #define DEBUG_GPIO_CLEAR(pin) db_gpio_clear(pin)
#else
    // No-op when DEBUG is not defined
    #define DEBUG_GPIO_TOGGLE(pin) ((void)0))
    #define DEBUG_GPIO_SET(pin) ((void)0))
#endif

//=========================== defines ==========================================

//=========================== variables ========================================

tsch_slot_timing_t tsch_default_slot_timing = {
    .rx_offset = 40 + 300, // Radio ramp-up time (40 us), + 100 us for any processing needed
    .rx_max = _TSCH_START_GUARD_TIME + _TSCH_PACKET_TOA_WITH_PADDING, // Guard time + Enough time to receive the maximum payload.
    .tx_offset = 40 + 300 + _TSCH_START_GUARD_TIME, // Same as rx_offset, plus the guard time.
    .tx_max = _TSCH_PACKET_TOA_WITH_PADDING, // Enough to transmit the maximum payload.
    .end_guard = _TSCH_END_GUARD_TIME, // Extra time at the end of the slot

    // receive slot is: rx_offset / rx_max / end_guard
    // transmit slot is: tx_offset / tx_max / end_guard
    .total_duration = 40 + 100 + _TSCH_START_GUARD_TIME + _TSCH_PACKET_TOA_WITH_PADDING + _TSCH_END_GUARD_TIME, // Total duration of the slot
};

typedef enum {
    // common,
    TSCH_STATE_BEGIN_SLOT,

    // receiver,
    TSCH_STATE_WAIT_RX_OFFSET = 11,
    TSCH_STATE_DO_RX = 12,
    TSCH_STATE_IS_RXING = 13,

    // transmitter,
    TSCH_STATE_WAIT_TX_OFFSET = 20,
    TSCH_STATE_DO_TX = 21,
    TSCH_STATE_IS_TXING = 22,

} tsch_state_t; // TODO: actually use this state in the handlers below

typedef struct {
    node_type_t node_type; // whether the node is a gateway or a dotbot
    uint64_t device_id; ///< Device ID

    uint64_t asn; ///< Absolute slot number

    tsch_state_t state; ///< State of the TSCH state machine
    tsch_radio_event_t event; ///< Current event to process

    uint8_t packet[DB_BLE_PAYLOAD_MAX_LENGTH]; ///< Buffer to store the packet to transmit
    size_t packet_len; ///< Length of the packet to transmit

    tsch_cb_t application_callback; ///< Function pointer, stores the application callback
} tsch_vars_t;

static tsch_vars_t _tsch_vars = { 0 };

uint8_t default_packet[] = {
    0x08, // size of the packet
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};

//========================== prototypes ========================================

//-------------------------- state machine handlers ----------------------------

static void _tsch_state_machine_handler(void);
static void _handler_sm_begin_slot(void);

/*
* @brief Set TSCH state machine state, to be used after the timer expires.
*/
static inline void _set_next_state(tsch_state_t state);

// ------------------------- other functions ------------------------------------

/*
* @brief Callback function passed to the radio driver.
*/
static void _tsch_callback(uint8_t *packet, uint8_t length);

static inline void _set_timer_and_compensate(uint8_t channel, uint32_t duration, uint32_t start_ts, timer_hf_cb_t cb);
// static inline void _set_timer(uint8_t channel, uint32_t duration, timer_hf_cb_t timer_callback);


//=========================== public ===========================================

void db_tsch_init(node_type_t node_type, tsch_cb_t application_callback) {
#ifdef DEBUG
    db_gpio_init(&pin0, DB_GPIO_OUT);
    db_gpio_init(&pin1, DB_GPIO_OUT);
    db_gpio_init(&pin2, DB_GPIO_OUT);
    db_gpio_init(&pin3, DB_GPIO_OUT);
#endif

    // initialize the high frequency clock
    db_timer_hf_init(TSCH_TIMER_DEV);

    // initialize the radio
    db_radio_init(&_tsch_callback, DB_RADIO_BLE_2MBit);  // set the radio callback to our tsch catch function

    // Save the application callback to use in our interruption
    _tsch_vars.application_callback = application_callback;

    // set node information
    _tsch_vars.node_type = node_type;
    _tsch_vars.device_id = db_device_id();

    _tsch_vars.asn = 0;

    // NOTE: assume the scheduler has already been initialized by the application

    // set slot total duration
    tsch_default_slot_timing.total_duration = tsch_default_slot_timing.rx_offset + tsch_default_slot_timing.rx_max + tsch_default_slot_timing.end_guard;

    // initialize and start the state machine
    _set_next_state(TSCH_STATE_BEGIN_SLOT);
    uint32_t time_padding = 10; // account for function call and interrupt latency
    db_timer_hf_set_oneshot_us(TSCH_TIMER_DEV, TSCH_TIMER_INTER_SLOT_CHANNEL, time_padding, _tsch_state_machine_handler); // trigger the state machine
}

//=========================== private ==========================================

// state machine handler
void _tsch_state_machine_handler(void) {
    uint32_t start_ts = db_timer_hf_now(TSCH_TIMER_DEV);
    // printf("State: %d\n", _tsch_vars.state);

    switch (_tsch_vars.state) {
        case TSCH_STATE_BEGIN_SLOT:
            DEBUG_GPIO_TOGGLE(&pin0);
            DEBUG_GPIO_SET(&pin1);
            // set the timer for the next slot TOTAL DURATION
            // _set_timer(TSCH_TIMER_INTER_SLOT_CHANNEL, tsch_default_slot_timing.total_duration, &_tsch_state_machine_handler);
            _set_timer_and_compensate(TSCH_TIMER_INTER_SLOT_CHANNEL, tsch_default_slot_timing.total_duration, start_ts, &_tsch_state_machine_handler);
            _handler_sm_begin_slot();
            break;
        case TSCH_STATE_DO_RX:
            DEBUG_GPIO_CLEAR(&pin1);
            DEBUG_GPIO_SET(&pin2);
            // update state
            _set_next_state(TSCH_STATE_IS_RXING);
            // receive packets
            db_radio_rx(); // remember: this always starts with before the actual transmission begins, i.e, rx_offset < tx_offset always holds
            _set_timer_and_compensate(TSCH_TIMER_INTRA_SLOT_CHANNEL, tsch_default_slot_timing.rx_max, start_ts, &_tsch_state_machine_handler);
            break;
        case TSCH_STATE_DO_TX:
            DEBUG_GPIO_CLEAR(&pin1);
            DEBUG_GPIO_SET(&pin2);
            // update state
            _set_next_state(TSCH_STATE_IS_TXING);
            // send the packet
            db_radio_tx(_tsch_vars.packet, _tsch_vars.packet_len);
            _set_timer_and_compensate(TSCH_TIMER_INTRA_SLOT_CHANNEL, tsch_default_slot_timing.tx_max, start_ts, &_tsch_state_machine_handler);
            break;
        // in case was receiving or sending, now just finish. timeslot will begin again because of the inter-slot timer
        case TSCH_STATE_IS_RXING:
        case TSCH_STATE_IS_TXING:
            DEBUG_GPIO_CLEAR(&pin2);
            // just disable the radio and set the next state
            db_radio_disable();
            _set_next_state(TSCH_STATE_BEGIN_SLOT);
            break;
        default:
            break;
    }
}

void _handler_sm_begin_slot(void) {
    uint32_t start_ts = db_timer_hf_now(TSCH_TIMER_DEV);

    uint32_t timer_duration = 0;

    tsch_radio_event_t event = db_scheduler_tick(_tsch_vars.asn++);
    // printf("  Event %c:   %c, %d    Slot duration: %d\n", event.slot_type, event.radio_action, event.frequency, tsch_default_slot_timing.total_duration);

    switch (event.radio_action) {
        case TSCH_RADIO_ACTION_TX:
            // configure radio
            db_radio_disable();
            db_radio_set_frequency(event.frequency);

            // update state
            _tsch_vars.event = event;
            _set_next_state(TSCH_STATE_DO_TX);

            // get the packet to tx and save in _tsch_vars
            // TODO: how to get a packet? decide based on _tsch_vars.node_type and event.slot_type
            //       could the event come with a packet? sometimes maybe? or would it be confusing?
            _tsch_vars.packet_len = sizeof(default_packet);
            memcpy(_tsch_vars.packet, default_packet, _tsch_vars.packet_len);

            // set timer duration to resume again after tx_offset
            timer_duration = tsch_default_slot_timing.tx_offset;
            _set_timer_and_compensate(TSCH_TIMER_INTRA_SLOT_CHANNEL, timer_duration, start_ts, &_tsch_state_machine_handler);
            break;
        case TSCH_RADIO_ACTION_RX:
            // configure radio
            db_radio_disable();
            db_radio_set_frequency(event.frequency);

            // update state
            _tsch_vars.event = event;
            _set_next_state(TSCH_STATE_DO_RX);

            // set timer duration to resume again after rx_offset
            timer_duration = tsch_default_slot_timing.rx_offset;
            _set_timer_and_compensate(TSCH_TIMER_INTRA_SLOT_CHANNEL, timer_duration, start_ts, &_tsch_state_machine_handler);
            break;
        case TSCH_RADIO_ACTION_SLEEP:
            // just disable the radio and do nothing, then come back for the next slot
            db_radio_disable();
            _set_next_state(TSCH_STATE_BEGIN_SLOT); // keep the same state
            // timer_duration = tsch_default_slot_timing.total_duration;
            break;
    }

    // _set_timer_and_compensate(TSCH_TIMER_INTRA_SLOT_CHANNEL, timer_duration, start_ts, &_tsch_state_machine_handler);
}

// --------------------- timers ---------------------

static inline void _set_next_state(tsch_state_t state) {
    _tsch_vars.state = state;
}

static inline void _set_timer_and_compensate(uint8_t channel, uint32_t duration, uint32_t start_ts, timer_hf_cb_t timer_callback) {
    uint32_t elapsed_ts = db_timer_hf_now(TSCH_TIMER_DEV) - start_ts;
    // printf("Setting timer for duration %d, compensating for elapsed %d gives: %d\n", duration, elapsed_ts, duration - elapsed_ts);
    db_timer_hf_set_oneshot_us(
        TSCH_TIMER_DEV,
        channel,
        duration - elapsed_ts,
        timer_callback
    );
}

//static inline void _set_timer(uint8_t channel, uint32_t duration, timer_hf_cb_t timer_callback) {
////    printf("Setting timer for duration %d\n", duration);
//   db_timer_hf_set_oneshot_us(
//       TSCH_TIMER_DEV,
//       channel,
//       duration,
//       timer_callback
//   );
//}

// --------------------- others ---------------------

static void _tsch_callback(uint8_t *packet, uint8_t length) {
    printf("Received packet of length %d\n", length);

    // Check if the packet is big enough to have a header
    if (length < sizeof(protocol_header_t)) {
        return;
    }

    // Check if it is a beacon
    if (packet[1] == TSCH_PACKET_TYPE_BEACON) {
        // _tsch_handle_beacon(packet, length);
    } else if (_tsch_vars.application_callback) {
        _tsch_vars.application_callback(packet, length);
    }
}
