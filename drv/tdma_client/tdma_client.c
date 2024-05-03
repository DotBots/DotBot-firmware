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

#include "tdma_client.h"
#include "clock.h"
#include "radio.h"
#include "rng.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"

//=========================== defines ==========================================

#if defined(BOARD_SAILBOT_V1)
#define TDMA_CLIENT_RADIO_APPLICATION SailBot
#else
#define TDMA_CLIENT_RADIO_APPLICATION DotBot
#endif

#define TDMA_CLIENT_DEFAULT_FRAME_DURATION 500000                              ///< Default duration of the tdma frame, in microseconds.
#define TDMA_CLIENT_DEFAULT_RX_START       0                                   ///< start of the , in microseconds.
#define TDMA_CLIENT_DEFAULT_RX_DURATION    TDMA_CLIENT_DEFAULT_FRAME_DURATION  ///< Default duration of the tdma frame, in microseconds.
#define TDMA_CLIENT_DEFAULT_TX_START       10000                               ///< Default duration of the tdma frame, in microseconds.
#define TDMA_CLIENT_DEFAULT_TX_DURATION    5000                                ///< Default duration of the tdma frame, in microseconds.
#define TDMA_CLIENT_HF_TIMER_CC_TX         TIMER_HF_CB_CHANS - 2               ///< Which timer channel will be used for the TX state machine.
#define TDMA_CLIENT_HF_TIMER_CC_RX         TIMER_HF_CB_CHANS - 3               ///< Which timer channel will be used for the RX state machine.
#define TDMA_CLIENT_MAX_DELAY_WITHOUT_TX   500000                              ///< Max amount of time that can pass without TXing anything
#define TDMA_CLIENT_RING_BUFFER_SIZE       10                                  ///< Amount of TX packets the buffer can contain
#define RADIO_MESSAGE_MAX_SIZE             255                                 ///< Size of buffers used for SPI communications
#define RADIO_TX_RAMP_UP_TIME              140                                 ///< time it takes the radio to start a transmission

typedef struct {
    uint8_t  buffer[TDMA_CLIENT_RING_BUFFER_SIZE][PAYLOAD_MAX_LENGTH];  ///< arrays of radio messages waiting to be sent
    uint32_t packet_length[TDMA_CLIENT_RING_BUFFER_SIZE];               ///< arrays of the lenght of the packets in the buffer
    uint8_t  writeIndex;                                                ///< Index for next write
    uint8_t  readIndex;                                                 ///< Index for next read
    uint8_t  count;                                                     ///< Number of arrays in buffer
} tdma_client_ring_buffer_t;

typedef struct {
    tdma_client_cb_t             callback;                              ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    tdma_client_table_t          tdma_client_table;                     ///< Timing table
    db_tdma_registration_state_t registration_flag;                     ///< flag marking if the DotBot is registered with the Gateway or not.
    db_tdma_rx_state_t           rx_flag;                               ///< flag marking if the DotBot's is receving or not.
    uint32_t                     last_tx_packet_timestamp;              ///< Timestamp of when the last packet was sent
    uint64_t                     device_id;                             ///< Device ID of the DotBot
    tdma_client_ring_buffer_t    tx_ring_buffer;                        ///< ring buffer to queue the outgoing packets
    uint8_t                      byte_onair_time;                       ///< How many microseconds it takes to send a byte of data
    uint8_t                      radio_buffer[RADIO_MESSAGE_MAX_SIZE];  ///< Internal buffer that contains the command to send (from buttons)
} tdma_client_vars_t;

//=========================== variables ========================================

static tdma_client_vars_t _tdma_client_vars = { 0 };

// Transform the ble mode into how many microseconds it takes to send a single byte.
uint8_t ble_mode_to_byte_time[] = {
    8,   // DB_RADIO_BLE_1MBit
    4,   // DB_RADIO_BLE_2MBit
    64,  // DB_RADIO_BLE_LR125Kbit
    16,  // DB_RADIO_BLE_LR500Kbit
};

//========================== prototypes ========================================

static void tdma_client_callback(uint8_t *packet, uint8_t length);

///< TDMA timer Interrupts
void timer_tx_interrupt(void);
void timer_rx_interrupt(void);

/**
 * @brief Updates the RX and TX timings for the TDMA table.
 *        DIrectly from an DB_PROTOCOL_TDMA_UPDATE_TABLE packet.
 *
 * @param[in] table       New table of TDMA timings
 */
void _protocol_tdma_set_table(const protocol_tdma_table_t *table);

/**
 * @brief Initialize the ring buffer for spi captures
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
void _message_rb_init(tdma_client_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   packet_length   length of the packet to send trough the radio
 */
void _message_rb_add(tdma_client_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length);

/**
 * @brief retreive the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   packet_length   length of the packet to send trough the radio
 */
bool _message_rb_get(tdma_client_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[out]   packet_sent   true is a packet was sent, false if no packet was sent.
 */
bool _message_rb_tx_queue(uint16_t max_tx_duration_us);

/**
 * @brief sends a keep_alive packet to the gateway
 *
 */
void _tx_keep_alive_message(void);

/**
 * @brief register with the gatway to request a TDMA slot
 *
 */
void _tx_tdma_register_message(void);

/**
 * @brief get a random delay between 100ms and 228ms in microseconds
 *        to change how often the dotbot advertises itself
 *
 */
uint32_t _get_random_delay_us(void);

//=========================== public ===========================================

void db_tdma_client_init(tdma_client_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq) {

    // Initialize high frequency clock
    db_timer_hf_init();

    // Initialize the ring buffer of outbound messages
    _message_rb_init(&_tdma_client_vars.tx_ring_buffer);

    // Initialize Radio
    db_radio_init(&tdma_client_callback, radio_mode);  // set the radio callback to our tdma catch function
    db_radio_set_frequency(radio_freq);                // Pass through the rest of the arguments
    db_radio_rx();                                     // start receving packets

    // Start the random number generator
    db_rng_init();

    // Retrieve the device ID.
    _tdma_client_vars.device_id = db_device_id();

    // Save the user callback to use in our interruption
    _tdma_client_vars.callback = callback;

    // Save the on-air byte time
    _tdma_client_vars.byte_onair_time = ble_mode_to_byte_time[radio_mode];

    // Set the default time table
    _tdma_client_vars.tdma_client_table.frame_duration = TDMA_CLIENT_DEFAULT_FRAME_DURATION;
    _tdma_client_vars.tdma_client_table.rx_start       = TDMA_CLIENT_DEFAULT_RX_START;
    _tdma_client_vars.tdma_client_table.rx_duration    = TDMA_CLIENT_DEFAULT_RX_DURATION;
    _tdma_client_vars.tdma_client_table.tx_start       = TDMA_CLIENT_DEFAULT_TX_START;
    _tdma_client_vars.tdma_client_table.tx_duration    = TDMA_CLIENT_DEFAULT_TX_DURATION;

    // Set the starting states
    _tdma_client_vars.registration_flag = DB_TDMA_CLIENT_UNREGISTERED;
    _tdma_client_vars.rx_flag           = DB_TDMA_CLIENT_RX_ON;

    // Configure the Timers
    _tdma_client_vars.last_tx_packet_timestamp = db_timer_hf_now();                                                // start the counter saving when was the last packet sent.
    db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_TX, TDMA_CLIENT_DEFAULT_TX_START, &timer_tx_interrupt);     // start advertising behaviour
    db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_RX, TDMA_CLIENT_DEFAULT_RX_DURATION, &timer_rx_interrupt);  // check RX timer once per frame.
}

void db_tdma_client_set_table(const tdma_client_table_t *table) {

    _tdma_client_vars.tdma_client_table.frame_duration = table->frame_duration;
    _tdma_client_vars.tdma_client_table.rx_start       = table->rx_start;
    _tdma_client_vars.tdma_client_table.rx_duration    = table->rx_duration;
    _tdma_client_vars.tdma_client_table.tx_start       = table->tx_start;
    _tdma_client_vars.tdma_client_table.tx_duration    = table->tx_duration;
}

void db_tdma_client_get_table(tdma_client_table_t *table) {

    table->frame_duration = _tdma_client_vars.tdma_client_table.frame_duration;
    table->rx_start       = _tdma_client_vars.tdma_client_table.rx_start;
    table->rx_duration    = _tdma_client_vars.tdma_client_table.rx_duration;
    table->tx_start       = _tdma_client_vars.tdma_client_table.tx_start;
    table->tx_duration    = _tdma_client_vars.tdma_client_table.tx_duration;
}

void db_tdma_client_tx(const uint8_t *packet, uint8_t length) {

    // Add packet to the output buffer
    _message_rb_add(&_tdma_client_vars.tx_ring_buffer, (uint8_t *)packet, length);
}

void db_tdma_client_flush(void) {
    // Use the normal function to send queue messages, but with a really long time
    // So that there is enough time to send everything.
    _message_rb_tx_queue(50000);
}

void db_tdma_client_empty(void) {

    // Reinitialize the ring buffer to erase it
    _message_rb_init(&_tdma_client_vars.tx_ring_buffer);
}

db_tdma_registration_state_t db_tdma_client_get_status(void) {
    return _tdma_client_vars.registration_flag;
}
//=========================== private ==========================================

void _protocol_tdma_set_table(const protocol_tdma_table_t *table) {

    _tdma_client_vars.tdma_client_table.frame_duration = table->frame_period;
    _tdma_client_vars.tdma_client_table.rx_start       = table->rx_start;
    _tdma_client_vars.tdma_client_table.rx_duration    = table->rx_duration;
    _tdma_client_vars.tdma_client_table.tx_start       = table->tx_start;
    _tdma_client_vars.tdma_client_table.tx_duration    = table->tx_duration;
}

void _message_rb_init(tdma_client_ring_buffer_t *rb) {
    rb->writeIndex = 0;
    rb->readIndex  = 0;
    rb->count      = 0;
}

void _message_rb_add(tdma_client_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length) {

    memcpy(rb->buffer[rb->writeIndex], data, PAYLOAD_MAX_LENGTH);
    rb->packet_length[rb->writeIndex] = packet_length;
    rb->writeIndex                    = (rb->writeIndex + 1) % TDMA_CLIENT_RING_BUFFER_SIZE;

    if (rb->count < TDMA_CLIENT_RING_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        rb->readIndex = (rb->readIndex + 1) % TDMA_CLIENT_RING_BUFFER_SIZE;
    }
}

bool _message_rb_get(tdma_client_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length) {
    if (rb->count <= 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, rb->buffer[rb->readIndex], PAYLOAD_MAX_LENGTH);
    *packet_length = rb->packet_length[rb->readIndex];
    rb->readIndex  = (rb->readIndex + 1) % TDMA_CLIENT_RING_BUFFER_SIZE;
    rb->count--;

    return true;
}

bool _message_rb_tx_queue(uint16_t max_tx_duration_us) {

    // initialize variables
    uint32_t start_tx_slot = db_timer_hf_now();
    uint8_t  length        = 0;
    uint8_t  packet[PAYLOAD_MAX_LENGTH];
    bool     packet_sent_flag = false;  ///< flag to keep track if a packet get sent during this function call

    // check if there is something to send
    if (_tdma_client_vars.tx_ring_buffer.count > 0) {

        // and send messages until queue is empty
        while (_tdma_client_vars.tx_ring_buffer.count > 0) {
            // retrieve the oldest packet from the queue
            bool error = _message_rb_get(&_tdma_client_vars.tx_ring_buffer, packet, &length);
            if (!error) {
                break;
            }
            // Compute if there is still time to send the packet [in microseconds]
            uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + length * _tdma_client_vars.byte_onair_time;
            // If there is time to send the packet, send it
            if (db_timer_hf_now() + tx_time - start_tx_slot < max_tx_duration_us) {
            
                // disable the radio, before sending.
                db_radio_disable();
                db_radio_tx(packet, length);
                packet_sent_flag = true;
            } else {  // otherwise, put the packet back in the queue and leave

                _message_rb_add(&_tdma_client_vars.tx_ring_buffer, packet, length);
                break;
            }
        }
        // Update the last packet timer
        _tdma_client_vars.last_tx_packet_timestamp = start_tx_slot;
    }

    // renable the Radio RX
    if (_tdma_client_vars.rx_flag == DB_TDMA_CLIENT_RX_ON) {
        db_radio_rx();
    }

    return packet_sent_flag;
}

void _tx_keep_alive_message(void) {

    db_protocol_header_to_buffer(_tdma_client_vars.radio_buffer, DB_BROADCAST_ADDRESS, TDMA_CLIENT_RADIO_APPLICATION, DB_PROTOCOL_ADVERTISEMENT);
    size_t length = sizeof(protocol_header_t);
    db_radio_disable();
    db_radio_tx(_tdma_client_vars.radio_buffer, length);
}

void _tx_tdma_register_message(void) {

    db_protocol_header_to_buffer(_tdma_client_vars.radio_buffer, DB_BROADCAST_ADDRESS, TDMA_CLIENT_RADIO_APPLICATION, DB_PROTOCOL_ADVERTISEMENT);
    size_t length = sizeof(protocol_header_t);
    db_radio_disable();
    db_radio_tx(_tdma_client_vars.radio_buffer, length);
}

uint32_t _get_random_delay_us(void) {

    // Change how often the message gets sent, between 100 and 228 ms.
    uint8_t random_value;
    db_rng_read(&random_value);
    return 100000 + (random_value >> 2) * 1000;
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
static void tdma_client_callback(uint8_t *packet, uint8_t length) {
    NRF_P1->OUTSET = 1 << 10;
    (void)length;
    uint8_t           *ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _tdma_client_vars.device_id) {
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
            uint8_t              *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
            protocol_tdma_table_t tdma_table;
            memcpy(&tdma_table, cmd_ptr, sizeof(protocol_tdma_table_t));
            const uint32_t next_period_start = tdma_table.next_period_start;

            // Update the TDMA table
            _protocol_tdma_set_table(&tdma_table);

            // Set the DotBot as registered
            if (_tdma_client_vars.registration_flag == DB_TDMA_CLIENT_UNREGISTERED) {
                _tdma_client_vars.registration_flag = DB_TDMA_CLIENT_REGISTERED;
            }

            // Update the state machine
            _tdma_client_vars.rx_flag = DB_TDMA_CLIENT_RX_WAIT;

            // Update the timer interrupts
            db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_TX, next_period_start + _tdma_client_vars.tdma_client_table.tx_start, &timer_tx_interrupt);
            db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_RX, next_period_start + _tdma_client_vars.tdma_client_table.rx_start, &timer_rx_interrupt);

        } break;

        case DB_PROTOCOL_TDMA_SYNC_FRAME:
        {
            // There is no payload for this packet
            // Only resync the timer if the DotBot has already been registered.
            if (_tdma_client_vars.registration_flag == DB_TDMA_CLIENT_REGISTERED) {

                // Calculate delay caused by the transmission of the Sync frame by the server
                uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + (sizeof(protocol_header_t) + sizeof(protocol_sync_frame_t)) * _tdma_client_vars.byte_onair_time;

                // Update the timer interrupts
                db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_TX, _tdma_client_vars.tdma_client_table.tx_start - tx_time, &timer_tx_interrupt);
                db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_RX, _tdma_client_vars.tdma_client_table.rx_start - tx_time, &timer_rx_interrupt);

                // Update the frame period
                uint8_t              *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
                protocol_sync_frame_t sync_frame;
                memcpy(&sync_frame, cmd_ptr, sizeof(protocol_sync_frame_t));
                uint32_t frame_period = sync_frame.frame_period;

                // Protect against receiving garbage
                if (frame_period > 0 && frame_period < 500000) {
                _tdma_client_vars.tdma_client_table.frame_duration = frame_period;
                // Also update the RX_duration, becaus ewe are working on ALWAYS_ON mode
                _tdma_client_vars.tdma_client_table.rx_duration = frame_period;
                }

                // Also update the RX_duration, becaus ewe are working on ALWAYS_ON mode

                // update the state machine
                _tdma_client_vars.rx_flag = DB_TDMA_CLIENT_RX_ON;

                // Enable radio RX
                db_radio_rx();
            }
        } break;

        // This is not a TDMA packet, send to the user callback
        default:
        {
            if (_tdma_client_vars.callback) {
                _tdma_client_vars.callback(packet, length);
            }
        }
    }
    NRF_P1->OUTCLR = 1 << 10;
}

/**
 * @brief Interruption handler for the TX state machine timer
 *
 */
void timer_tx_interrupt(void) {

    NRF_P0->OUTSET   = 1 << 26;
    bool packet_sent = false;

    // Check the state of the device.
    if (_tdma_client_vars.registration_flag == DB_TDMA_CLIENT_REGISTERED) {

        // Prepare right now the next timer interruption
        db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_TX, _tdma_client_vars.tdma_client_table.frame_duration, &timer_tx_interrupt);

        // send messages if available
        packet_sent = _message_rb_tx_queue(_tdma_client_vars.tdma_client_table.tx_duration);

        // if no packet has been sent for a while, send a keep_alive ping to maintain the connection.
        if (!packet_sent) {
            if (db_timer_hf_now() - _tdma_client_vars.last_tx_packet_timestamp > TDMA_CLIENT_MAX_DELAY_WITHOUT_TX) {

                _tx_keep_alive_message();
                // Save the timestamp of the last packet
                _tdma_client_vars.last_tx_packet_timestamp = db_timer_hf_now();
            }
        }
    } else {  // Device is unregistered

        // Prepare right now the next timer interruption.
        uint32_t delay_time = _get_random_delay_us();
        db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_TX, delay_time, &timer_tx_interrupt);

        // Try to register with the TDMA server
        _tx_tdma_register_message();
        // Save the timestamp of the last packet
        _tdma_client_vars.last_tx_packet_timestamp = db_timer_hf_now();
    }
    NRF_P0->OUTCLR = 1 << 26;
}

/**
 * @brief Interruption handler for the RX state machine timer
 *
 */
void timer_rx_interrupt(void) {

    // If the duration of the RX timer is equal to the frame duration
    // just leave the radio ON permanently

    if (_tdma_client_vars.tdma_client_table.rx_duration == _tdma_client_vars.tdma_client_table.frame_duration) {
        db_radio_rx();

        return;

    } else {
        // Check if we just came from an idle period or an rx period
        if (_tdma_client_vars.rx_flag == DB_TDMA_CLIENT_RX_WAIT) {
            // Set the next interruption
            db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_RX, _tdma_client_vars.tdma_client_table.rx_duration, &timer_rx_interrupt);
            // turn the radio ON
            db_radio_rx();
        } else if (_tdma_client_vars.rx_flag == DB_TDMA_CLIENT_RX_ON) {
            // Set the next interruption
            uint32_t delay = _tdma_client_vars.tdma_client_table.frame_duration - _tdma_client_vars.tdma_client_table.rx_duration;
            db_timer_hf_set_oneshot_us(TDMA_CLIENT_HF_TIMER_CC_RX, delay, &timer_rx_interrupt);
            // turn the radio OFF
            db_radio_disable();
        }
    }

}