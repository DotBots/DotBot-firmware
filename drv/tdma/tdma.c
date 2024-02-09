/**
 * @file
 * @ingroup bsp_radio
 *
 * @brief  nRF52833-specific definition of the "radio" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "tdma.h"
#include "clock.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"

//=========================== defines ==========================================

#define TDMA_DEFAULT_FRAME_DURATION 500000                       ///< Default duration of the tdma frame, in microseconds.
#define TDMA_DEFAULT_RX_START       0                            ///< start of the , in microseconds.
#define TDMA_DEFAULT_RX_DURATION    TDMA_DEFAULT_FRAME_DURATION  ///< Default duration of the tdma frame, in microseconds.
#define TDMA_DEFAULT_TX_START       10000                        ///< Default duration of the tdma frame, in microseconds.
#define TDMA_DEFAULT_TX_DURATION    5000                         ///< Default duration of the tdma frame, in microseconds.
#define TDMA_HF_TIMER_CC_TX         TIMER_HF_CB_CHANS            ///< Which timer channel will be used for the TX state machine.
#define TDMA_HF_TIMER_CC_RX         TIMER_HF_CB_CHANS - 1        ///< Which timer channel will be used for the RX state machine.
#define TDMA_MAX_DELAY_WITHOUT_TX   500000                       ///< Max amount of time that can pass without TXing anything
#define TDMA_RING_BUFFER_SIZE       10                           ///< Amount of TX packets the buffer can contain
#define RADIO_MESSAGE_MAX_SIZE      64                           ///< Size of buffers used for SPI communications
#define TX_RAMP_UP_TIME             140                          ///< time it takes the radio to start a transmission

typedef struct {
    uint8_t  buffer[TDMA_RING_BUFFER_SIZE][PAYLOAD_MAX_LENGTH];  ///< arrays of radio messages waiting to be sent
    uint32_t packet_length[TDMA_RING_BUFFER_SIZE];               ///< arrays of the lenght of the packets in the buffer
    uint8_t  writeIndex;                                         ///< Index for next write
    uint8_t  readIndex;                                          ///< Index for next read
    uint8_t  count;                                              ///< Number of arrays in buffer
} tdma_ring_buffer_t;

typedef struct {
    tdma_cb_t                    callback;                  ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    tdma_table_t                 tdma_table;                ///< Timing table
    db_tdma_registration_state_t registration_flag;         ///< flag marking if the DotBot is registered with the Gateway or not.
    db_tdma_tx_state_t           tx_flag;                   ///< flag marking if the DotBot's radio is transmitting or not.
    db_tdma_rx_state_t           rx_flag;                   ///< flag marking if the DotBot's is receving or not.
    uint32_t                     last_tx_packet_timestamp;  ///< How many microseconds since the last packet was sent
    uint64_t                     device_id;                 ///< Device ID of the DotBot
    tdma_ring_buffer_t           tx_ring_buffer;            ///< ring buffer to queue the outgoing packets
    uint8_t                      byte_onair_time;           ///< How many microseconds it takes to send a byte of data
} tdma_vars_t;

//=========================== variables ========================================

tdma_vars_t _tdma_vars = { 0 };

// Transform the ble mode into how many microseconds it takes to send a single byte.
uint8_t ble_mode_to_byte_time[] = {
    8,   // DB_RADIO_BLE_1MBit
    4,   // DB_RADIO_BLE_2MBit
    64,  // DB_RADIO_BLE_LR125Kbit
    16,  // DB_RADIO_BLE_LR500Kbit
};

//========================== prototypes ========================================

static void tdma_callback(uint8_t *packet, uint8_t length);

///< TDMA timer Interrupts
void timer_tx_interrupt(void);
void timer_rx_interrupt(void);

///< TX behaviour when DotBot is unregistered.
void ftm_tx_1(void);

/**
 * @brief Initialize the ring buffer for spi captures
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
void _init_tdma_ring_buffer(tdma_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   packet_length   length of the packet to send trough the radio
 */
void _add_to_tdma_ring_buffer(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length);

/**
 * @brief retreive the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   packet_length   length of the packet to send trough the radio
 */
bool _get_from_tdma_ring_buffer(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 * 
 * @param[out]   packet_sent   true is a packet was sent, false if no packet was sent.
 */
bool _send_queued_messages(void);

//=========================== public ===========================================

void db_tdma_init(tdma_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq, uint8_t buffer_size) {

    // Initialize Radio
    db_radio_init(&tdma_callback, radio_mode);  // set the radio callback to our tdma catch function
    db_radio_set_frequency(radio_freq);         // Pass through the rest of the arguments
    db_radio_rx();                              // start receving packets

    // Retrieve the device ID.
    _tdma_vars.device_id = db_device_id();

    // Save the user callback to use in our interruption
    _tdma_vars.callback = callback;

    // Save the on-air byte time
    _tdma_vars.byte_onair_time = ble_mode_to_byte_time[radio_mode];

    // Set the default time table
    _tdma_vars.tdma_table.frame       = TDMA_DEFAULT_FRAME_DURATION;
    _tdma_vars.tdma_table.rx_start    = TDMA_DEFAULT_RX_START;
    _tdma_vars.tdma_table.rx_duration = TDMA_DEFAULT_RX_DURATION;
    _tdma_vars.tdma_table.tx_start    = TDMA_DEFAULT_TX_START;
    _tdma_vars.tdma_table.tx_duration = TDMA_DEFAULT_TX_DURATION;

    // Set the starting states
    _tdma_vars.registration_flag = DB_TDMA_UNREGISTERED;
    _tdma_vars.tx_flag           = DB_TDMA_TX_WAIT;
    _tdma_vars.rx_flag           = DB_TDMA_RX_ON;

    // Configure the Timers
    _tdma_vars.last_tx_packet_timestamp = db_timer_hf_now();                                         // start the counter saving when was the last packet sent.
    db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, TDMA_DEFAULT_TX_START, &timer_tx_interrupt);     // start advertising behaviour
    db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_RX, TDMA_DEFAULT_RX_DURATION, &timer_rx_interrupt);  // check RX timer once per frame.
}

void db_tdma_update_table(tdma_table_t *table) {

    _tdma_vars.tdma_table.frame       = table->frame;
    _tdma_vars.tdma_table.rx_start    = table->rx_start;
    _tdma_vars.tdma_table.rx_duration = table->rx_duration;
    _tdma_vars.tdma_table.tx_start    = table->tx_start;
    _tdma_vars.tdma_table.tx_duration = table->tx_duration;
}

//=========================== private ==========================================

void _init_tdma_ring_buffer(tdma_ring_buffer_t *rb) {
    rb->writeIndex = 0;
    rb->readIndex  = 0;
    rb->count      = 0;
}

void _add_to_tdma_ring_buffer(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length) {

    memcpy(rb->buffer[rb->writeIndex], data, PAYLOAD_MAX_LENGTH);
    rb->packet_length[rb->writeIndex] = packet_length;
    rb->writeIndex                    = (rb->writeIndex + 1) % TDMA_RING_BUFFER_SIZE;

    if (rb->count < TDMA_RING_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        rb->readIndex = (rb->readIndex + 1) % TDMA_RING_BUFFER_SIZE;
    }
}

bool _get_from_tdma_ring_buffer(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length) {
    if (rb->count <= 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, rb->buffer[rb->readIndex], PAYLOAD_MAX_LENGTH);
    *packet_length = rb->packet_length[rb->readIndex];
    rb->readIndex  = (rb->readIndex + 1) % TDMA_RING_BUFFER_SIZE;
    rb->count--;

    return true;
}

bool _send_queued_messages(void){

    // initialize variables
    uint32_t start_tx_slot = db_timer_hf_now();
    uint8_t pending_packets = _tdma_vars.tx_ring_buffer.count;
    uint8_t length          = 0;
    uint8_t packet[PAYLOAD_MAX_LENGTH];
    bool packet_sent_flag = false;              ///< flag to keep track if a packet get sent during this function call

    // check if there is something to send
    if (_tdma_vars.tx_ring_buffer.count > 0) {
        // if yes, disable the radio.
        db_radio_disable();
        // and send messages until queue is empty
        while (_tdma_vars.tx_ring_buffer.count > 0) {
            // retrieve the oldest packet from the queue
            bool error = _get_from_spi_ring_buffer(&_tdma_vars.tx_ring_buffer, packet, &length);
            if (!error) {
                break;
            }
            // Compute if there is still time to send the packet [in microseconds]
            uint16_t tx_time = 140 + length * _tdma_vars.byte_onair_time;
            // If there is time to send the packet, send it
            if (db_timer_hf_now() + tx_time - start_tx_slot < _tdma_vars.tdma_table.tx_duration) {

                db_radio_tx(packet, length);
            }
            else { // otherwise, put the packet back in the queue and leave

                _add_to_tdma_ring_buffer(&_tdma_vars.tx_ring_buffer, packet, length);
                break;
            }
        }
        // Update the last packet timer
        _tdma_vars.last_tx_packet_timestamp = start_tx_slot;

    } else {
        // if too long have passed without sending anything, send a KEPP_ALIVE message
        if (start_tx_slot - _tdma_vars.last_tx_packet_timestamp >= TDMA_MAX_DELAY_WITHOUT_TX) {
            //TODO: send a keep alive message.
        } 

    }

    // renable the Radio RX 
    if (_tdma_vars.rx_flag == DB_TDMA_RX_ON) {
        db_radio_rx();
        
    }
}

//=========================== interrupt handlers ===============================

/**
 * @brief Interruption handler for the Radio.
 *
 * This function will be called each time a radio packet is received.
 * it will catch any TDMA related packet and update the timing table.
 * All other types of packets are passed directly into the user-defined call-back
 *
 */
static void tdma_callback(uint8_t *packet, uint8_t length) {

    (void)length;
    uint8_t           *ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _tdma_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // THe application type needs to be checked inside the application itself.
    // We don't know it apriori

    // check and process the TDMA packets
    switch (header->type) {
        case DB_PROTOCOL_TDMA_UPDATE_TABLE:
        {
            // Get the payload
            uint8_t            *cmd_ptr           = ptk_ptr + sizeof(protocol_header_t);
            const uint32_t      next_period_start = ((const protocol_tdma_table_t *)cmd_ptr)->next_period_start;
            const tdma_table_t *tdma_table        = (const tdma_table_t *)cmd_ptr;

            // Update the TDMA table
            db_tdma_update_table(tdma_table);

            // Set the DotBot as registered
            if (_tdma_vars.registration_flag == DB_TDMA_UNREGISTERED) {
                _tdma_vars.registration_flag = DB_TDMA_REGISTERED;
            }

            // Update the state machine
            _tdma_vars.tx_flag = DB_TDMA_TX_WAIT;
            _tdma_vars.rx_flag = DB_TDMA_RX_WAIT;

            // Update the timer interrupts
            db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, next_period_start + _tdma_vars.tdma_table.tx_start, &timer_tx_interrupt);
            db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_RX, next_period_start + _tdma_vars.tdma_table.rx_start, &timer_rx_interrupt);

        } break;

        case DB_PROTOCOL_TDMA_SYNC_FRAME:
        {
            // There is no payload for this packet

            // Only resync the timer if the DotBot has already been registered.
            if (_tdma_vars.registration_flag == DB_TDMA_REGISTERED) {

                // update the state machine
                _tdma_vars.tx_flag = DB_TDMA_TX_WAIT;
                _tdma_vars.rx_flag = DB_TDMA_RX_WAIT;

                // Enable radio RX
                db_radio_rx();

                // Update the timer interrupts
                db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, _tdma_vars.tdma_table.tx_start, &timer_tx_interrupt);
                db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_RX, _tdma_vars.tdma_table.rx_start, &timer_rx_interrupt);
            }
        } break;

        // This is not a TDMA packet, send to the user callback
        default:
        {
            if (_tdma_vars.callback) {
                _tdma_vars.callback(packet, length);
            }
        }
    }
}

/**
 * @brief Interruption handler for the TX state machine timer
 *
 */
void timer_tx_interrupt(void) {

    bool packet_sent = false;

    // Check the state of the device.
    if (_tdma_vars.registration_flag == DB_TDMA_REGISTERED) {

        if (_tdma_vars.tx_flag == DB_TDMA_TX_WAIT) {

            // Prepare right now the next timer interruption
            db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, _tdma_vars.tdma_table.frame, &timer_tx_interrupt);

            // send messages if available
            packet_sent = _send_queued_messages();

            if (!packet_sent) {
                //TODO: add the keep_alive function calculation in here in here 
            }


        } else {
        }


        // check if no message was sent, and more than 500ms have passed. Then send a keep_alive beacon

    } else  // Device is unregistered
    {
        // TODO send an advertisement beacon.
    }
}