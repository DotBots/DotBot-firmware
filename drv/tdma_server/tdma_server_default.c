/**
 * @file
 * @ingroup     drv_tdma_server
 *
 * @brief       Driver for Time-Division-Multiple-Access for the Gateway radio
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2024
 */
#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "tdma_server.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"
#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "ipc.h"
#endif
//=========================== defines ==========================================

#define TDMA_SERVER_DEFAULT_FRAME_DURATION_US 20000  ///< Default duration of the tdma frame, in microseconds.

#define TDMA_SERVER_DEFAULT_RX_START_US    0                                      ///< Start of RX slot, in microseconds.
#define TDMA_SERVER_DEFAULT_RX_DURATION_US TDMA_SERVER_DEFAULT_FRAME_DURATION_US  ///< Duration of RX slot, in microseconds.
#define TDMA_SERVER_DEFAULT_TX_START_US    0                                      ///< Start of TX slot, in microseconds.
#define TDMA_SERVER_DEFAULT_TX_DURATION_US TDMA_SERVER_TIME_SLOT_DURATION_US      ///< Duration of TX slot, in microseconds.

#define TDMA_SERVER_HF_TIMER_CC      0       ///< Which timer channel will be used for the TX state machine.
#define TDMA_MAX_DELAY_WITHOUT_TX    500000  ///< Max amount of time that can pass without TXing anything
#define TDMA_RING_BUFFER_SIZE        10      ///< Amount of TX packets the buffer can contain
#define TDMA_NEW_CLIENT_BUFFER_SIZE  30      ///< Amount of clients waiting to register the buffer can contain
#define RADIO_MESSAGE_MAX_SIZE       255     ///< Size of buffers used for SPI communications
#define RADIO_TX_RAMP_UP_TIME        140     ///< time it takes the radio to start a transmission
#define TDMA_TX_DEADTIME_US          100     ///< buffer time between tdma slot to avoid accidentally sen
#define TDMA_SERVER_CLIENT_NOT_FOUND -1      ///< The client is not registered in the server's table.

#define TDMA_SERVER_TIMER_HF 2

typedef struct {
    uint8_t  buffer[TDMA_RING_BUFFER_SIZE][DB_BLE_PAYLOAD_MAX_LENGTH];  ///< arrays of radio messages waiting to be sent
    uint32_t packet_length[TDMA_RING_BUFFER_SIZE];                      ///< arrays of the length of the packets in the buffer
    uint8_t  write_index;                                               ///< Index for next write
    uint8_t  read_index;                                                ///< Index for next read
    uint8_t  count;                                                     ///< Number of arrays in buffer
} tdma_ring_buffer_t;

typedef struct {
    uint64_t buffer[TDMA_NEW_CLIENT_BUFFER_SIZE];  ///< arrays of client IDs waiting to register
    uint8_t  write_index;                          ///< Index for next write
    uint8_t  read_index;                           ///< Index for next read
    uint8_t  count;                                ///< Number of clients in the buffer
} new_client_ring_buffer_t;

typedef struct {
    tdma_server_cb_t         callback;                              ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    tdma_server_table_t      tdma_table;                            ///< Timing table
    uint16_t                 active_slot_idx;                       ///< index of the current active slot in the TDMA table
    uint32_t                 last_tx_packet_ts;                     ///< Timestamp of when the previous packet was sent
    uint32_t                 frame_start_ts;                        ///< Timestamp of when the previous tdma superframe started
    uint32_t                 slot_start_ts;                         ///< Timestamp of when the current tdma slot started
    uint8_t                  byte_onair_time;                       ///< How many microseconds it takes to send a byte of data
    uint64_t                 device_id;                             ///< Device ID of the DotBot
    tdma_ring_buffer_t       tx_ring_buffer;                        ///< ring buffer to queue the outgoing packets
    uint8_t                  radio_buffer[RADIO_MESSAGE_MAX_SIZE];  ///< Internal buffer that contains the command to send (from buttons)
    new_client_ring_buffer_t new_clients_rb;                        //
} tdma_server_vars_t;

//=========================== variables ========================================

static tdma_server_vars_t _tdma_vars = { 0 };

// Transform the ble mode into how many microseconds it takes to send a single byte.
static const uint8_t ble_mode_to_byte_time[] = {
    8,   // DB_RADIO_BLE_1MBit
    4,   // DB_RADIO_BLE_2MBit
    64,  // DB_RADIO_BLE_LR125Kbit
    16,  // DB_RADIO_BLE_LR500Kbit
};

//========================== prototypes ========================================

static void tdma_server_callback(uint8_t *packet, uint8_t length);

/// TDMA timer Interrupts
static void timer_tdma_interrupt(void);

/**
 * @brief Initialize the ring buffer for spi captures
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
static void _message_rb_init(tdma_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   packet_length   length of the packet to send trough the radio
 */
static void _message_rb_add(tdma_ring_buffer_t *rb, uint8_t data[DB_BLE_PAYLOAD_MAX_LENGTH], uint8_t packet_length);

/**
 * @brief retrieve the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   packet_length   length of the packet to send trough the radio
 */
static bool _message_rb_get(tdma_ring_buffer_t *rb, uint8_t data[DB_BLE_PAYLOAD_MAX_LENGTH], uint8_t *packet_length);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[in]    rb                     pointer to ring buffer structure
 * @param[in]    max_tx_duration_us     max time available to send messages.
 * @return                              true if a packet was sent, false if no packet was sent.
 */
static bool _message_rb_tx_queue(tdma_ring_buffer_t *rb, uint16_t max_tx_duration_us);

/**
 * @brief Initialize the ring buffer for clients waiting to register.
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
static void _client_rb_init(new_client_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   new_client  id of the client waiting to register.
 */
static void _client_rb_add(new_client_ring_buffer_t *rb, uint64_t new_client);

/**
 * @brief retrieve the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   new_client  pointer to the variable where the new client ID will be saved.
 * @return                   true if ID was successfully copied, false if buffer is empty.
 */
static bool _client_rb_get(new_client_ring_buffer_t *rb, uint64_t *new_client);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[in]    rb                  pointer to ring buffer structure
 * @param[in]    max_tx_duration_us  max time available to send messages.
 * @return                           true if a packet was sent, false if no packet was sent.
 */
static bool _client_rb_tx_queue(new_client_ring_buffer_t *rb, uint16_t max_tx_duration_us);

/**
 * @brief searches if a client id already has a reminder message queued up
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   client      ID client to search.
 * @return                   true if ID was found, false otherwise.
 */
static bool _client_rb_id_exists(new_client_ring_buffer_t *rb, uint64_t client);

/**
 * @brief Send a message to synchronize the client's and the server's clock
 *
 */
static void _tx_sync_frame(void);

/**
 * @brief Send the sync packet + tdma_slot parameters to a particular client.
 *
 * @param[in] client    MAC ID of the client to register
 *
 */
static void _tx_registration_messages(uint64_t client);

/**
 * @brief find the slot in which a client is registered
 *
 * @param[in]   tdma_table  pointer to the tdma table to search
 * @param[in]   client      id of the client to search in the table.
 * @return table slot index of the client, or -1 if client is not found.
 */
static int16_t _server_find_client(tdma_server_table_t *tdma_table, uint64_t client);

/**
 * @brief register a new client into the table.
 *
 * @param[in]   tdma_table  pointer to the tdma table to search
 * @param[in]   client      id of the client to register.
 */
static void _server_register_new_client(tdma_server_table_t *tdma_table, uint64_t client);

//=========================== public ===========================================

void db_tdma_server_init(tdma_server_cb_t callback, db_radio_mode_t radio_mode, uint8_t radio_freq) {

    // Initialize high frequency clock
    db_timer_hf_init(TDMA_SERVER_TIMER_HF);

    // Initialize the ring buffer of outbound messages
    _message_rb_init(&_tdma_vars.tx_ring_buffer);

    // Initialize the client buffer of outbound messages
    _client_rb_init(&_tdma_vars.new_clients_rb);

    // Initialize Radio
    db_radio_init(&tdma_server_callback, radio_mode);  // set the radio callback to our tdma catch function
    db_radio_set_frequency(radio_freq);                // Pass through the rest of the arguments
    db_radio_rx();                                     // start receiving packets

    // Retrieve the device ID.
    _tdma_vars.device_id = db_device_id();

    // Save the user callback to use in our interruption
    _tdma_vars.callback = callback;

    // Save the on-air byte time
    _tdma_vars.byte_onair_time = ble_mode_to_byte_time[radio_mode];

    // Set the default time table, and populate the first entry with the server
    _tdma_vars.tdma_table.frame_duration_us    = TDMA_SERVER_DEFAULT_FRAME_DURATION_US;
    _tdma_vars.tdma_table.table[0].client      = _tdma_vars.device_id;
    _tdma_vars.tdma_table.table[0].rx_start    = TDMA_SERVER_DEFAULT_RX_START_US;
    _tdma_vars.tdma_table.table[0].rx_duration = TDMA_SERVER_DEFAULT_RX_DURATION_US;
    _tdma_vars.tdma_table.table[0].tx_start    = TDMA_SERVER_DEFAULT_TX_START_US;
    _tdma_vars.tdma_table.table[0].tx_duration = TDMA_SERVER_DEFAULT_TX_DURATION_US;

    // set the current active slot
    _tdma_vars.active_slot_idx = 0;

    // Configure the Timers
    _tdma_vars.last_tx_packet_ts = db_timer_hf_now(TDMA_SERVER_TIMER_HF);                                                                                                    // start the counter saving when was the last packet sent.
    _tdma_vars.frame_start_ts    = _tdma_vars.last_tx_packet_ts;                                                                                                             // start the counter saving when was the last packet sent.
    db_timer_hf_set_periodic_us(TDMA_SERVER_TIMER_HF, TDMA_SERVER_HF_TIMER_CC, _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration, &timer_tdma_interrupt);  // start advertising behaviour
}

void db_tdma_server_get_table_info(uint32_t *frame_duration_us, uint16_t *num_clients, uint16_t *table_index) {

    *frame_duration_us = _tdma_vars.tdma_table.frame_duration_us;
    *num_clients       = _tdma_vars.tdma_table.num_clients;
    *table_index       = _tdma_vars.tdma_table.table_index;
}

void db_tdma_server_get_client_info(tdma_table_entry_t *client, uint8_t client_id) {

    // Copy and return one entry of the TDMA entry
    memcpy(client, &_tdma_vars.tdma_table.table[client_id], sizeof(tdma_table_entry_t));
}

void db_tdma_server_tx(const uint8_t *packet, uint8_t length) {
    // Add packet to the output buffer
    _message_rb_add(&_tdma_vars.tx_ring_buffer, (uint8_t *)packet, length);
}

void db_tdma_server_flush(void) {
    // Use the normal function to send queue messages, but with a really long time
    // So that there is enough time to send everything.
    _message_rb_tx_queue(&_tdma_vars.tx_ring_buffer, 50000);
}

void db_tdma_server_empty(void) {

    // Reinitialize the ring buffer to erase it
    _message_rb_init(&_tdma_vars.tx_ring_buffer);
}

//=========================== private ==========================================

static void _message_rb_init(tdma_ring_buffer_t *rb) {
    rb->write_index = 0;
    rb->read_index  = 0;
    rb->count       = 0;
}

static void _message_rb_add(tdma_ring_buffer_t *rb, uint8_t data[DB_BLE_PAYLOAD_MAX_LENGTH], uint8_t packet_length) {

    memcpy(rb->buffer[rb->write_index], data, DB_BLE_PAYLOAD_MAX_LENGTH);
    rb->packet_length[rb->write_index] = packet_length;
    rb->write_index                    = (rb->write_index + 1) % TDMA_RING_BUFFER_SIZE;

    if (rb->count < TDMA_RING_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust read_index
        rb->read_index = (rb->read_index + 1) % TDMA_RING_BUFFER_SIZE;
    }
}

static bool _message_rb_get(tdma_ring_buffer_t *rb, uint8_t data[DB_BLE_PAYLOAD_MAX_LENGTH], uint8_t *packet_length) {
    if (rb->count == 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, rb->buffer[rb->read_index], DB_BLE_PAYLOAD_MAX_LENGTH);
    *packet_length = rb->packet_length[rb->read_index];
    rb->read_index = (rb->read_index + 1) % TDMA_RING_BUFFER_SIZE;
    rb->count--;

    return true;
}

static bool _message_rb_tx_queue(tdma_ring_buffer_t *rb, uint16_t max_tx_duration_us) {

    // initialize variables
    uint8_t length                            = 0;
    uint8_t packet[DB_BLE_PAYLOAD_MAX_LENGTH] = { 0 };
    bool    packet_sent_flag                  = false;  ///< flag to keep track if a packet get sent during this function call

    // Return if there is nothing to send
    if (rb->count == 0) {
        return false;
    }

    // Otherwise, send messages until queue is empty
    while (rb->count > 0) {
        // retrieve the oldest packet from the queue
        bool error = _message_rb_get(rb, packet, &length);
        if (!error) {
            break;
        }
        // Compute if there is still time to send the packet [in microseconds]
        uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + length * _tdma_vars.byte_onair_time;
        // If there is time to send the packet, send it
        if (db_timer_hf_now(TDMA_SERVER_TIMER_HF) + tx_time - _tdma_vars.slot_start_ts < max_tx_duration_us) {
            // switch off RX, and send message.
            db_radio_disable();
            db_radio_tx(packet, length);
            packet_sent_flag = true;
        } else {  // otherwise, put the packet back in the queue and leave

            _message_rb_add(rb, packet, length);
            break;
        }
    }
    // Update the last packet timer
    _tdma_vars.last_tx_packet_ts = _tdma_vars.slot_start_ts;

    return packet_sent_flag;
}

static void _client_rb_init(new_client_ring_buffer_t *rb) {
    rb->write_index = 0;
    rb->read_index  = 0;
    rb->count       = 0;
}

static void _client_rb_add(new_client_ring_buffer_t *rb, uint64_t new_client) {

    rb->buffer[rb->write_index] = new_client;
    rb->write_index             = (rb->write_index + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;

    if (rb->count < TDMA_NEW_CLIENT_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust read_index
        rb->read_index = (rb->read_index + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    }
}

static bool _client_rb_get(new_client_ring_buffer_t *rb, uint64_t *new_client) {
    if (rb->count == 0) {
        // Buffer is empty
        return false;
    }

    *new_client    = rb->buffer[rb->read_index];
    rb->read_index = (rb->read_index + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    rb->count--;

    return true;
}

static bool _client_rb_tx_queue(new_client_ring_buffer_t *rb, uint16_t max_tx_duration_us) {

    // initialize variables
    uint64_t client           = 0;
    bool     packet_sent_flag = false;  ///< flag to keep track if a packet get sent during this function call

    // check if there is something to send
    if (rb->count > 0) {
        // and send messages until queue is empty
        while (rb->count > 0) {
            // retrieve the oldest packet from the queue
            bool error = _client_rb_get(rb, &client);
            if (!error) {
                break;
            }
            // Compute if there is still time to send the packet [in microseconds]
            uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + (sizeof(protocol_header_t) + sizeof(protocol_tdma_table_t)) * _tdma_vars.byte_onair_time;
            // If there is time to send the packet, send it
            if (db_timer_hf_now(TDMA_SERVER_TIMER_HF) + tx_time - _tdma_vars.slot_start_ts < max_tx_duration_us) {
                _tx_registration_messages(client);
                packet_sent_flag = true;
            } else {  // otherwise, put the packet back in the queue and leave

                _client_rb_add(rb, client);
                break;
            }
        }
        // Update the last packet timer
        _tdma_vars.last_tx_packet_ts = _tdma_vars.slot_start_ts;
    }
    return packet_sent_flag;
}

static bool _client_rb_id_exists(new_client_ring_buffer_t *rb, uint64_t client) {

    // get the raw pointer for the ring buffer.
    // check all valid values

    if (rb->count == 0) {
        // Buffer is empty
        return false;
    }

    for (size_t i = rb->read_index; i != rb->write_index; i = (i + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE) {
        if (rb->buffer[i] == client) {
            return true;
        }
    }

    return false;
}

static void _tx_sync_frame(void) {
    // This message signals the start of a TDMA frame
    // Prepare packet payload
    protocol_sync_frame_t frame = { _tdma_vars.tdma_table.frame_duration_us };
    // Prepare packet header
    size_t length = db_protocol_tdma_sync_frame_to_buffer(_tdma_vars.radio_buffer, DB_BROADCAST_ADDRESS, &frame);
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

static void _tx_registration_messages(uint64_t client) {
    // Send the sync packet + tdma_slot parameters to a particular client.

    // Create the TDMA table we will send
    protocol_tdma_table_t table = { 0 };

    // Get the info of the client we are sending too.
    int16_t slot = _server_find_client(&_tdma_vars.tdma_table, client);

    // global frame data
    table.frame_period = _tdma_vars.tdma_table.frame_duration_us;
    // Client specific data
    table.rx_duration = _tdma_vars.tdma_table.table[slot].rx_duration;
    table.rx_start    = _tdma_vars.tdma_table.table[slot].rx_start;
    table.tx_duration = _tdma_vars.tdma_table.table[slot].tx_duration;
    table.tx_start    = _tdma_vars.tdma_table.table[slot].tx_start;

    // Compute the time before the next frame. (as close as possible to the TX as you can, so that it's more accurate)
    table.next_period_start = table.frame_period - (db_timer_hf_now(TDMA_SERVER_TIMER_HF) - _tdma_vars.frame_start_ts);

    // Fill out the buffer with the TDMA message (header + table)
    size_t length = db_protocol_tdma_table_update_to_buffer(_tdma_vars.radio_buffer, client, &table);

    // Send the message
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

static int16_t _server_find_client(tdma_server_table_t *tdma_table, uint64_t client) {

    for (size_t i = 0; i <= tdma_table->table_index; i++) {
        if (tdma_table->table[i].client == client) {
            return i;
        }
    }
    return TDMA_SERVER_CLIENT_NOT_FOUND;
}

static void _server_register_new_client(tdma_server_table_t *tdma_table, uint64_t client) {

    // Check if the next slot should go to the gateway
    //  YES: Assign next+1 slot to the new client
    //  NO: Assign next slot.

    // Check if the next index of the table should be a gateway slot.
    if (((tdma_table->table_index + 1) % (int)(TDMA_SERVER_MAX_GATEWAY_TX_DELAY_US / TDMA_SERVER_TIME_SLOT_DURATION_US)) == 0) {

        // Compute the new frame duration knowing that we will add two new slots to the table (gateway + client)
        uint32_t frame_duration = ((tdma_table->table_index + 2) + 1) * TDMA_SERVER_DEFAULT_TX_DURATION_US;
        // if the new frame is smaller than the minimum frame time, keep the minimum frame time.
        frame_duration = (frame_duration > TDMA_SERVER_DEFAULT_FRAME_DURATION_US) ? frame_duration : TDMA_SERVER_DEFAULT_FRAME_DURATION_US;

        // first slot belongs to the gateway.
        tdma_table->table_index += 1;
        uint8_t idx = tdma_table->table_index;  // use a shorter variable to make the code more understandable

        tdma_table->table[idx].client      = _tdma_vars.device_id;
        tdma_table->table[idx].rx_start    = TDMA_SERVER_DEFAULT_RX_START_US;
        tdma_table->table[idx].rx_duration = frame_duration;
        tdma_table->table[idx].tx_start    = (idx)*TDMA_SERVER_DEFAULT_TX_DURATION_US;
        tdma_table->table[idx].tx_duration = TDMA_SERVER_DEFAULT_TX_DURATION_US;

        // second slot belongs to the client.
        tdma_table->table_index += 1;
        idx = tdma_table->table_index;  // use a shorter variable to make the code more understandable

        tdma_table->table[idx].client      = client;
        tdma_table->table[idx].rx_start    = TDMA_SERVER_DEFAULT_RX_START_US;
        tdma_table->table[idx].rx_duration = frame_duration;
        tdma_table->table[idx].tx_start    = (idx)*TDMA_SERVER_DEFAULT_TX_DURATION_US;
        tdma_table->table[idx].tx_duration = TDMA_SERVER_DEFAULT_TX_DURATION_US;
        // Update the frame duration
        tdma_table->frame_duration_us = frame_duration;
    } else {
        // first slot belongs to the client.
        tdma_table->table_index += 1;
        uint8_t idx = tdma_table->table_index;  // use a shorter variable to make the code more understandable

        // Compute the new frame duration knowing that we will add one new slots to the table (client)
        uint32_t frame_duration = (idx + 1) * TDMA_SERVER_DEFAULT_TX_DURATION_US;
        // if the new frame is smaller than the minimum frame time, keep the minimum frame time.
        frame_duration = (frame_duration > TDMA_SERVER_DEFAULT_FRAME_DURATION_US) ? frame_duration : TDMA_SERVER_DEFAULT_FRAME_DURATION_US;

        tdma_table->table[idx].client      = client;
        tdma_table->table[idx].rx_start    = TDMA_SERVER_DEFAULT_RX_START_US;
        tdma_table->table[idx].rx_duration = frame_duration;
        tdma_table->table[idx].tx_start    = (idx)*TDMA_SERVER_DEFAULT_TX_DURATION_US;
        tdma_table->table[idx].tx_duration = TDMA_SERVER_DEFAULT_TX_DURATION_US;

        // Update the frame duration
        tdma_table->frame_duration_us = frame_duration;
    }

    // Update the last slot, table index, number of clients, in the TDMA table
    tdma_table->num_clients += 1;
}

//=========================== interrupt handlers ===============================

/**
 * @brief Interruption handler for the Radio.
 *
 * This function will be called each time a radio packet is received.
 * it will catch any TDMA related packet and update the timing table.
 * All other types of packets are passed directly into the user-defined call-back
 *
 * @param[in]   packet    pointer to the data array with the data packet
 * @param[in]   length    length of the packet received trough the radio
 */
static void tdma_server_callback(uint8_t *packet, uint8_t length) {

    /*
    - Check message sender is in the TDMA table
        NO:
            - Assign it a slot in the table.
            - Record in the ringbuffer to send the registration message  (ON TIMER)
                - check that it's not already in the ring buffer waiting to go be sent.
        YES:
            - Is it in the correct slot?
                NO:
                    - Record in the ringbuffer to send a reminder message. (ON TIMER)
                YES:
                    Pass up to PyDotBot

    */

    uint8_t           *ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // Check destination address matches
    if (header->dst != DB_GATEWAY_ADDRESS && header->dst != _tdma_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Handle unregistered DotBot
    if (_server_find_client(&_tdma_vars.tdma_table, header->src) == TDMA_SERVER_CLIENT_NOT_FOUND) {

        // register new client to the table
        _server_register_new_client(&_tdma_vars.tdma_table, header->src);
        // Put it in the list of clients to transmit to in your next turn.
        _client_rb_add(&_tdma_vars.new_clients_rb, header->src);

    } else {

        // Handle Out-of-Slot messages
        uint64_t current_client = _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].client;
        if (current_client != header->src) {

            // Check that you don't already have a a reminder queued up for this client
            if (!_client_rb_id_exists(&_tdma_vars.new_clients_rb, header->src)) {

                // Put it in the list of clients to transmit to in your next turn.
                _client_rb_add(&_tdma_vars.new_clients_rb, header->src);
            }
        }
    }

    // Consume TDMA-only messages, don't let it go up to the application.
    if (header->packet_type == DB_PACKET_TDMA_KEEP_ALIVE) {
        return;
    }

    // Pipe the message to the user
    if (_tdma_vars.callback) {
        _tdma_vars.callback(packet, length);
    }
}

/**
 * @brief Interruption handler for the TX state machine timer
 *
 */
static void timer_tdma_interrupt(void) {

    // Save the timestamp start of the current slot, to ensure accurate computation of the end of the slot
    _tdma_vars.slot_start_ts = db_timer_hf_now(TDMA_SERVER_TIMER_HF);

    // If there are not enough clients, make sure that the minimum frame time is respected, even if is just filled with empty slots.
    uint8_t last_slot;
    if (_tdma_vars.tdma_table.frame_duration_us > TDMA_SERVER_DEFAULT_FRAME_DURATION_US) {
        // enough clients, wrap around the end of the list
        last_slot = _tdma_vars.tdma_table.table_index + 1;
    } else {
        // not enough clients, use the default amount of slots for the default frame duration
        last_slot = (TDMA_SERVER_DEFAULT_FRAME_DURATION_US / TDMA_SERVER_TIME_SLOT_DURATION_US);
    }
    // Update the active client for this slot. Wrap around after the end of the active clients is reached.
    _tdma_vars.active_slot_idx = (_tdma_vars.active_slot_idx + 1) % (last_slot);

    // check if this is the start of the super frame to send a sync message.
    if (_tdma_vars.active_slot_idx == 0) {

        // Update last-superframe timestamp
        _tdma_vars.frame_start_ts = _tdma_vars.slot_start_ts;

        // Send a resync frame
        _tx_sync_frame();
    }

    bool packet_sent = false;

    // Check that it's your timeslot
    if (_tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].client == _tdma_vars.device_id) {

        // Send registration messages. + Out of slot messages. (Use AT MOST, half of the available slot time.)
        uint32_t remaining_slot_time_us = _tdma_vars.slot_start_ts + _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration - db_timer_hf_now(TDMA_SERVER_TIMER_HF);
        packet_sent                     = _client_rb_tx_queue(&_tdma_vars.new_clients_rb, remaining_slot_time_us / 2 - TDMA_TX_DEADTIME_US);

        // send messages if available. time_available (slot_start + slot_duration - current_time)
        remaining_slot_time_us = _tdma_vars.slot_start_ts + _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration - db_timer_hf_now(TDMA_SERVER_TIMER_HF);
        packet_sent            = _message_rb_tx_queue(&_tdma_vars.tx_ring_buffer, remaining_slot_time_us - TDMA_TX_DEADTIME_US);

        // mark last time you sent anything
        if (packet_sent) {
            _tdma_vars.last_tx_packet_ts = db_timer_hf_now(TDMA_SERVER_TIMER_HF);
        }
    }
}
