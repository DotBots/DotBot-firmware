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

#include "tdma_server.h"
#include "clock.h"
#include "radio.h"
#include "timer_hf.h"
#include "protocol.h"
#include "device.h"

//=========================== defines ==========================================

#if defined(BOARD_SAILBOT_V1)
#define TDMA_RADIO_APPLICATION SailBot
#else
#define TDMA_RADIO_APPLICATION DotBot
#endif

#define TDMA_SERVER_DEFAULT_FRAME_DURATION_US 20000  ///< Default duration of the tdma frame, in microseconds.

#define TDMA_SERVER_DEFAULT_RX_START_US    0                                      ///< start of the , in microseconds.
#define TDMA_SERVER_DEFAULT_RX_DURATION_US TDMA_SERVER_DEFAULT_FRAME_DURATION_US  ///< Default duration of the tdma frame, in microseconds.
#define TDMA_SERVER_DEFAULT_TX_START_US    0                                      ///< Default duration of the tdma frame, in microseconds.
#define TDMA_SERVER_DEFAULT_TX_DURATION_US TDMA_SERVER_TIME_SLOT_DURATION_US      ///< Default duration of the tdma frame, in microseconds.

#define TDMA_SERVER_HF_TIMER_CC      TIMER_HF_CB_CHANS - 2  ///< Which timer channel will be used for the TX state machine.
#define TDMA_MAX_DELAY_WITHOUT_TX    500000             ///< Max amount of time that can pass without TXing anything
#define TDMA_RING_BUFFER_SIZE        10                 ///< Amount of TX packets the buffer can contain
#define TDMA_NEW_CLIENT_BUFFER_SIZE  30                 ///< Amount of clients waiting to register the buffer can contain
#define RADIO_MESSAGE_MAX_SIZE       255                ///< Size of buffers used for SPI communications
#define RADIO_TX_RAMP_UP_TIME        140                ///< time it takes the radio to start a transmission
#define TDMA_TX_DEADTIME_US          100                ///< buffer time between tdma slot to avoid accidentally sen
#define TDMA_SERVER_CLIENT_NOT_FOUND -1                 ///< The client is not registered in the server's table.

typedef struct {
    uint8_t  buffer[TDMA_RING_BUFFER_SIZE][PAYLOAD_MAX_LENGTH];  ///< arrays of radio messages waiting to be sent
    uint32_t packet_length[TDMA_RING_BUFFER_SIZE];               ///< arrays of the lenght of the packets in the buffer
    uint8_t  writeIndex;                                         ///< Index for next write
    uint8_t  readIndex;                                          ///< Index for next read
    uint8_t  count;                                              ///< Number of arrays in buffer
} tdma_ring_buffer_t;

typedef struct {
    uint64_t buffer[TDMA_NEW_CLIENT_BUFFER_SIZE];  ///< arrays of client IDs waiting to register
    uint8_t  writeIndex;                           ///< Index for next write
    uint8_t  readIndex;                            ///< Index for next read
    uint8_t  count;                                ///< Number of clients in the buffer
} new_client_ring_buffer_t;

typedef struct {
    tdma_server_cb_t    callback;                              ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    tdma_server_table_t tdma_table;                            ///< Timing table
    uint16_t            active_slot_idx;                       ///< index of the current active slot in the TDMA table
    uint32_t            last_tx_packet_ts;                     ///< Timestamp of when the previous packet was sent
    uint32_t            frame_start_ts;                        ///< Timestamp of when the previous tdma superframe started
    uint32_t            slot_start_ts;                         ///< Timestamp of when the current tdma slot started
    uint8_t             byte_onair_time;                       ///< How many microseconds it takes to send a byte of data
    uint64_t            device_id;                             ///< Device ID of the DotBot
    tdma_ring_buffer_t  tx_ring_buffer;                        ///< ring buffer to queue the outgoing packets
    uint8_t             radio_buffer[RADIO_MESSAGE_MAX_SIZE];  ///< Internal buffer that contains the command to send (from buttons)

    new_client_ring_buffer_t new_clients_rb;  //
} tdma_server_vars_t;

//=========================== variables ========================================

static tdma_server_vars_t _tdma_vars = { 0 };

// Transform the ble mode into how many microseconds it takes to send a single byte.
uint8_t ble_mode_to_byte_time[] = {
    8,   // DB_RADIO_BLE_1MBit
    4,   // DB_RADIO_BLE_2MBit
    64,  // DB_RADIO_BLE_LR125Kbit
    16,  // DB_RADIO_BLE_LR500Kbit
};

//========================== prototypes ========================================

static void tdma_server_callback(uint8_t *packet, uint8_t length);

///< TDMA timer Interrupts
void timer_tdma_interrupt(void);

/**
 * @brief Initialize the ring buffer for spi captures
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
void _message_rb_init(tdma_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   packet_length   length of the packet to send trough the radio
 */
void _message_rb_add(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length);

/**
 * @brief retreive the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   packet_length   length of the packet to send trough the radio
 */
bool _message_rb_get(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[in]    rb                  pointer to ring buffer structure
 * @param[in]    max_tx_duration_us     max time available to send messages.
 * @return                              true is a packet was sent, false if no packet was sent.
 */
bool _message_rb_tx_queue(tdma_ring_buffer_t *rb, uint16_t max_tx_duration_us);

/**
 * @brief Initialize the ring buffer for clients waiting to register.
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
void _client_rb_init(new_client_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   new_client  id of the client waiting to register.
 */
void _client_rb_add(new_client_ring_buffer_t *rb, uint64_t new_client);

/**
 * @brief retreive the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   new_client  pointer to the variable where the new client ID will be saved.
 * @return                   true if ID was successfully copied, false buffer is empty.
 */
bool _client_rb_get(new_client_ring_buffer_t *rb, uint64_t *new_client);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[in]    rb                  pointer to ring buffer structure
 * @param[in]    max_tx_duration_us  max time available to send messages.
 * @return                           true is a packet was sent, false if no packet was sent.
 */
bool _client_rb_tx_queue(new_client_ring_buffer_t *rb, uint16_t max_tx_duration_us);

/**
 * @brief searches if a client id already has a reminder message queued up
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   client      ID client to search.
 * @return                   true if ID was found, false otherwise.
 */
bool _client_rb_id_exists(new_client_ring_buffer_t *rb, uint64_t client);

/**
 * @brief Send a message to synchronize the client's and the server's clock
 *
 */
void _tx_sync_frame(void);

/**
 * @brief respond to all clients requesting a registration,
 *        or correct those that stray away from their slot
 *
 */
void _tx_registration_messages(uint64_t client);

/**
 * @brief find the slot in which a client is registered
 *
 * @param[in]   tdma_table  pointer to the tdma table to search
 * @param[in]   client      id of the client to search in the table.
 * @return table slot index of the client, or -1 if client is not found.
 */
uint8_t _server_find_client(tdma_server_table_t *tdma_table, uint64_t client);

/**
 * @brief register a new client into the table.
 *
 * @param[in]   tdma_table  pointer to the tdma table to search
 * @param[in]   client      id of the client to register.
 */
void _server_register_new_client(tdma_server_table_t *tdma_table, uint64_t client);

//=========================== public ===========================================

void db_tdma_server_init(tdma_server_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq) {

    // Initialize high frequency clock
    db_timer_hf_init();

    // Initialize the ring buffer of outbound messages
    _message_rb_init(&_tdma_vars.tx_ring_buffer);

    // Initialize the client buffer of outbound messages
    _client_rb_init(&_tdma_vars.new_clients_rb);

    // Initialize Radio
    db_radio_init(&tdma_server_callback, radio_mode);  // set the radio callback to our tdma catch function
    db_radio_set_frequency(radio_freq);                // Pass through the rest of the arguments
    db_radio_rx();                                     // start receving packets

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
    _tdma_vars.last_tx_packet_ts = db_timer_hf_now();                                                                                                  // start the counter saving when was the last packet sent.
    _tdma_vars.frame_start_ts    = _tdma_vars.last_tx_packet_ts;                                                                                       // start the counter saving when was the last packet sent.
    db_timer_hf_set_periodic_us(TDMA_SERVER_HF_TIMER_CC, _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration, &timer_tdma_interrupt);  // start advertising behaviour
}

tdma_server_table_t *db_tdma_server_get_table(void) {

    return &_tdma_vars.tdma_table;
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

void _message_rb_init(tdma_ring_buffer_t *rb) {
    rb->writeIndex = 0;
    rb->readIndex  = 0;
    rb->count      = 0;
}

void _message_rb_add(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t packet_length) {

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

bool _message_rb_get(tdma_ring_buffer_t *rb, uint8_t data[PAYLOAD_MAX_LENGTH], uint8_t *packet_length) {
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

bool _message_rb_tx_queue(tdma_ring_buffer_t *rb, uint16_t max_tx_duration_us) {

    // initialize variables
    uint8_t length = 0;
    uint8_t packet[PAYLOAD_MAX_LENGTH];
    bool    packet_sent_flag = false;  ///< flag to keep track if a packet get sent during this function call

    // check if there is something to send
    if (rb->count > 0) {
        // if yes, disable the radio.
        db_radio_disable();
        // and send messages until queue is empty
        while (rb->count > 0) {
            // retrieve the oldest packet from the queue
            bool error = _message_rb_get(rb, packet, &length);
            if (!error) {
                break;
            }
            // Compute if there is still time to send the packet [in microseconds]
            uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + length * _tdma_vars.byte_onair_time;
            // If there is time to send the packet, send it
            if (db_timer_hf_now() + tx_time - _tdma_vars.slot_start_ts < max_tx_duration_us) {

                db_radio_tx(packet, length);
                packet_sent_flag = true;
            } else {  // otherwise, put the packet back in the queue and leave

                _message_rb_add(rb, packet, length);
                break;
            }
        }
        // Update the last packet timer
        _tdma_vars.last_tx_packet_ts = _tdma_vars.slot_start_ts;
    }
    return packet_sent_flag;
}

void _client_rb_init(new_client_ring_buffer_t *rb) {
    rb->writeIndex = 0;
    rb->readIndex  = 0;
    rb->count      = 0;
}

void _client_rb_add(new_client_ring_buffer_t *rb, uint64_t new_client) {

    rb->buffer[rb->writeIndex] = new_client;
    rb->writeIndex             = (rb->writeIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;

    if (rb->count < TDMA_NEW_CLIENT_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        rb->readIndex = (rb->readIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    }
}

bool _client_rb_get(new_client_ring_buffer_t *rb, uint64_t *new_client) {
    if (rb->count <= 0) {
        // Buffer is empty
        return false;
    }

    *new_client   = rb->buffer[rb->readIndex];
    rb->readIndex = (rb->readIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    rb->count--;

    return true;
}

bool _client_rb_tx_queue(new_client_ring_buffer_t *rb, uint16_t max_tx_duration_us) {

    // initialize variables
    uint64_t client           = 0;
    bool     packet_sent_flag = false;  ///< flag to keep track if a packet get sent during this function call

    // check if there is something to send
    if (rb->count > 0) {
        // if yes, disable the radio.
        db_radio_disable();
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
            if (db_timer_hf_now() + tx_time - _tdma_vars.slot_start_ts < max_tx_duration_us) {

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

bool _client_rb_id_exists(new_client_ring_buffer_t *rb, uint64_t client) {

    // get the raw pointer for the ring buffer.
    // check all valid values

    if (rb->count <= 0) {
        // Buffer is empty
        return false;
    }

    for (size_t i = rb->readIndex; i != rb->writeIndex; i = (i + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE) {
        if (rb->buffer[i] == client) {
            return true;
        }
    }

    return false;
}

void _tx_sync_frame(void) {
    // This message signals the start of a TDMA frame
    // Prepare packet
    protocol_sync_frame_t frame = { _tdma_vars.tdma_table.frame_duration_us };
    // Send the packet
    db_protocol_header_to_buffer(_tdma_vars.radio_buffer, DB_BROADCAST_ADDRESS, TDMA_RADIO_APPLICATION, DB_PROTOCOL_TDMA_SYNC_FRAME);
    memcpy(_tdma_vars.radio_buffer + sizeof(protocol_header_t), &frame, sizeof(protocol_sync_frame_t));
    size_t length = sizeof(protocol_header_t);
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

void _tx_registration_messages(uint64_t client) {
    // Send the sync packet to a particular client.

    // Create the TDMA table we will send
    protocol_tdma_table_t table = { 0 };

    // Get the info of the client we are sending too.
    uint8_t slot = _server_find_client(&_tdma_vars.tdma_table, client);

    // global frame data
    table.frame_period = _tdma_vars.tdma_table.frame_duration_us;
    // Client specific data
    table.rx_duration = _tdma_vars.tdma_table.table[slot].rx_duration;
    table.rx_start    = _tdma_vars.tdma_table.table[slot].rx_start;
    table.tx_duration = _tdma_vars.tdma_table.table[slot].tx_duration;
    table.tx_start    = _tdma_vars.tdma_table.table[slot].tx_start;

    // Start filling out the Message header
    db_protocol_header_to_buffer(_tdma_vars.radio_buffer, client, TDMA_RADIO_APPLICATION, DB_PROTOCOL_TDMA_UPDATE_TABLE);

    // Compute the time before the next frame. (as close as possible to the TX as you can, so that it's more accurate)
    table.next_period_start = table.frame_period - (db_timer_hf_now() - _tdma_vars.frame_start_ts);

    // Fill the rest of the message.
    memcpy(_tdma_vars.radio_buffer + sizeof(protocol_header_t), &table, sizeof(protocol_tdma_table_t));
    size_t length = sizeof(protocol_header_t) + sizeof(protocol_tdma_table_t);

    // Send the message
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

uint8_t _server_find_client(tdma_server_table_t *tdma_table, uint64_t client) {

    for (size_t i = 0; i <= tdma_table->table_index; i++) {
        if (tdma_table->table[i].client == client) {
            return i;
        }
    }
    return false;
}

void _server_register_new_client(tdma_server_table_t *tdma_table, uint64_t client) {

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
        uint8_t idx = tdma_table->table_index;  // use a shorter variable to make  the code more understandable

        tdma_table->table[idx].client      = _tdma_vars.device_id;
        tdma_table->table[idx].rx_start    = TDMA_SERVER_DEFAULT_RX_START_US;
        tdma_table->table[idx].rx_duration = frame_duration;
        tdma_table->table[idx].tx_start    = (idx)*TDMA_SERVER_DEFAULT_TX_DURATION_US;
        tdma_table->table[idx].tx_duration = TDMA_SERVER_DEFAULT_TX_DURATION_US;

        // second slot belongs to the client.
        tdma_table->table_index += 1;
        idx = tdma_table->table_index;  // use a shorter variable to make  the code more understandable

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
        uint8_t idx = tdma_table->table_index;  // use a shorter variable to make  the code more understandable

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

    // Handle unregistered DotBot
    if (!_server_find_client(&_tdma_vars.tdma_table, header->src)) {

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

    // Pipe the message to the user
    if (_tdma_vars.callback) {
        _tdma_vars.callback(packet, length);
    }
}

/**
 * @brief Interruption handler for the TX state machine timer
 *
 */
void timer_tdma_interrupt(void) {

    // Save the timestamp start of the current slot, to ensure accurate computation of the end of the slot
    _tdma_vars.slot_start_ts = db_timer_hf_now();

    // If there are not enough clients, make sure the the minimum frame time is respected, even if is just filled with empty slots.
    uint8_t last_slot;
    if (_tdma_vars.tdma_table.frame_duration_us > TDMA_SERVER_DEFAULT_FRAME_DURATION_US) {
        // enough clients, wrap around the end of the list
        last_slot = _tdma_vars.tdma_table.table_index;
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

        // Send registration messages. + Out of slot messages. (Use AT MOST, helf of the available slot time.)
        uint32_t remaining_slot_time_us = _tdma_vars.slot_start_ts + _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration - db_timer_hf_now();
        packet_sent                     = _client_rb_tx_queue(&_tdma_vars.new_clients_rb, remaining_slot_time_us / 2 - TDMA_TX_DEADTIME_US);

        // send messages if available. time_available (slot_start + slot_duration - current_time)
        remaining_slot_time_us = _tdma_vars.slot_start_ts + _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration - db_timer_hf_now();
        packet_sent            = _message_rb_tx_queue(&_tdma_vars.tx_ring_buffer, remaining_slot_time_us - TDMA_TX_DEADTIME_US);

        // mark last time you sent anything
        if (packet_sent) {
            _tdma_vars.last_tx_packet_ts = db_timer_hf_now();
        }
    }
}
