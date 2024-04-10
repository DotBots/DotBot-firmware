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

#define TDMA_SERVER_HF_TIMER_CC     TIMER_HF_CB_CHANS  ///< Which timer channel will be used for the TX state machine.
#define TDMA_MAX_DELAY_WITHOUT_TX   500000             ///< Max amount of time that can pass without TXing anything
#define TDMA_RING_BUFFER_SIZE       10                 ///< Amount of TX packets the buffer can contain
#define TDMA_NEW_CLIENT_BUFFER_SIZE 30                 ///< Amount of clients waiting to register the buffer can contain
#define RADIO_MESSAGE_MAX_SIZE      255                ///< Size of buffers used for SPI communications
#define RADIO_TX_RAMP_UP_TIME       140                ///< time it takes the radio to start a transmission
#define TDMA_TX_DEADTIME_US         100                ///< buffer time between tdma slot to avoid accidentally sen

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
    tdma_server_cb_t    callback;           ///< Function pointer, stores the callback to use in the RADIO_Irq handler.
    tdma_server_table_t tdma_table;         ///< Timing table
    uint16_t            active_slot_idx;    ///< index of the current active slot in the TDMA table
    uint32_t            last_tx_packet_ts;  ///< Timestamp of when the previous packet was sent
    uint32_t            frame_start_ts;     ///< Timestamp of when the previous tdma superframe started
    uint32_t            slot_start_ts;      ///< Timestamp of when the current tdma slot started

    uint64_t           device_id;                             ///< Device ID of the DotBot
    tdma_ring_buffer_t tx_ring_buffer;                        ///< ring buffer to queue the outgoing packets
    uint8_t            radio_buffer[RADIO_MESSAGE_MAX_SIZE];  ///< Internal buffer that contains the command to send (from buttons)

    new_client_ring_buffer_t new_clients_rb;  //
} tdma_server_vars_t;

//=========================== variables ========================================

tdma_server_vars_t _tdma_vars = { 0 };

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
void timer_tdma_interrupt(void);

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
 * @brief Initialize the ring buffer for clients waiting to register.
 *
 * @param[in]   rb  pointer to ring buffer structure
 */
void _init_client_ring_buffer(new_client_ring_buffer_t *rb);

/**
 * @brief add one element to the ring buffer for tdma captures
 *
 * @param[in]   rb          pointer to ring buffer structure
 * @param[in]   new_client  id of the client waiting to register.
 */
void _add_to_client_ring_buffer(new_client_ring_buffer_t *rb, uint64_t new_client);

/**
 * @brief retreive the oldest element from the ring buffer for tdma captures
 *
 * @param[in]    rb          pointer to ring buffer structure
 * @param[out]   new_client  pointer to the variable where the new client ID will be saved.
 * @return                   true if ID was successfully copied, false buffer is empty.
 */
bool _get_from_client_ring_buffer(new_client_ring_buffer_t *rb, uint64_t *new_client);

/**
 * @brief Sends all the queued messages that can be sent in during the TX timeslot
 *
 * @param[out]   packet_sent   true is a packet was sent, false if no packet was sent.
 */
bool _send_queued_messages(uint16_t max_tx_duration_us);

/**
 * @brief sends a keep_alive packet to the gateway
 *
 */
void _send_keep_alive_message(void);

/**
 * @brief Send a message to synchronize the client's and the server's clock
 *
 */
void _send_sync_frame(void);

/**
 * @brief respond to all clients requesting a registration,
 *        or correct those that stray away from their slot
 *
 */
void _send_registration_messages(void);

//=========================== public ===========================================

void db_tdma_init(tdma_server_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq, uint8_t buffer_size) {

    // Initialize the ring buffer of outbound messages
    _init_tdma_ring_buffer(&_tdma_vars.tx_ring_buffer);

    // Initialize the client buffer of outbound messages
    _init_client_ring_buffer(&_tdma_vars.new_clients_rb);

    // Initialize Radio
    db_radio_init(&tdma_server_callback, radio_mode);  // set the radio callback to our tdma catch function
    db_radio_set_frequency(radio_freq);                // Pass through the rest of the arguments
    db_radio_rx();                                     // start receving packets

    // Retrieve the device ID.
    _tdma_vars.device_id = db_device_id();

    // Save the user callback to use in our interruption
    _tdma_vars.callback = callback;

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

void db_tdma_update_table(tdma_table_t *table) {

    _tdma_vars.tdma_table.frame       = table->frame;
    _tdma_vars.tdma_table.rx_start    = table->rx_start;
    _tdma_vars.tdma_table.rx_duration = table->rx_duration;
    _tdma_vars.tdma_table.tx_start    = table->tx_start;
    _tdma_vars.tdma_table.tx_duration = table->tx_duration;
}

void db_tdma_flush(void) {
    // Use the normal function to send queue messages, but with a really long time
    // So that there is enough time to send everything.
    _send_queued_messages(50000);
}

void db_tdma_empty(void) {

    // Reinitialize the ring buffer to erase it
    _init_tdma_ring_buffer(&_tdma_vars.tx_ring_buffer);
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

void _init_client_ring_buffer(new_client_ring_buffer_t *rb) {
    rb->writeIndex = 0;
    rb->readIndex  = 0;
    rb->count      = 0;
}

void _add_to_client_ring_buffer(new_client_ring_buffer_t *rb, uint64_t new_client) {

    rb->buffer[rb->writeIndex] = new_client;
    rb->writeIndex             = (rb->writeIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;

    if (rb->count < TDMA_NEW_CLIENT_BUFFER_SIZE) {
        rb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        rb->readIndex = (rb->readIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    }
}

bool _get_from_client_ring_buffer(new_client_ring_buffer_t *rb, uint64_t *new_client) {
    if (rb->count <= 0) {
        // Buffer is empty
        return false;
    }

    *new_client   = rb->buffer[rb->readIndex];
    rb->readIndex = (rb->readIndex + 1) % TDMA_NEW_CLIENT_BUFFER_SIZE;
    rb->count--;

    return true;
}

bool _send_queued_messages(uint16_t max_tx_duration_us) {

    // initialize variables
    uint32_t start_tx_slot = db_timer_hf_now();
    uint8_t  length        = 0;
    uint8_t  packet[PAYLOAD_MAX_LENGTH];
    bool     packet_sent_flag = false;  ///< flag to keep track if a packet get sent during this function call

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
            uint16_t tx_time = RADIO_TX_RAMP_UP_TIME + length * _tdma_vars.byte_onair_time;
            // If there is time to send the packet, send it
            if (db_timer_hf_now() + tx_time - start_tx_slot < max_tx_duration_us) {

                db_radio_tx(packet, length);
            } else {  // otherwise, put the packet back in the queue and leave

                _add_to_tdma_ring_buffer(&_tdma_vars.tx_ring_buffer, packet, length);
                break;
            }
        }
        // Update the last packet timer
        _tdma_vars.last_tx_packet_ts = start_tx_slot;
    }

    // renable the Radio RX
    if (_tdma_vars.rx_flag == DB_TDMA_RX_ON) {
        db_radio_rx();
    }
}

void _send_keep_alive_message(void) {

    db_protocol_header_to_buffer(_tdma_vars.radio_buffer, DB_BROADCAST_ADDRESS, TDMA_RADIO_APPLICATION, DB_PROTOCOL_TDMA_KEEP_ALIVE);
    size_t length = sizeof(protocol_header_t);
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

void _send_sync_frame(void) {

    db_protocol_header_to_buffer(_tdma_vars.radio_buffer, DB_BROADCAST_ADDRESS, TDMA_RADIO_APPLICATION, DB_PROTOCOL_TDMA_SYNC_FRAME);
    size_t length = sizeof(protocol_header_t);
    db_radio_disable();
    db_radio_tx(_tdma_vars.radio_buffer, length);
}

void _register_new_client(void){
    // TODO: calculate the new positions in the tdma for the new client, and update it.
    // Calculate if more slots for the server are needed.
    // Check if the "new client" already exists to not modify the table

    // Broadcast the new frame duration.
    // Send the registration message to the specific dotbot.
}

void _send_registration_messages(void) {
    // TODO: Send the sync packet to a particular client.
    //  Calculate the correct time to send the the slot start timer.
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

    // The application type needs to be checked inside the application itself.
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
            _tdma_vars.rx_flag = DB_TDMA_RX_WAIT;

            // Update the timer interrupts
            db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, next_period_start + _tdma_vars.tdma_table.tx_start, &timer_tx_interrupt);

        } break;

        case DB_PROTOCOL_TDMA_SYNC_FRAME:
        {
            // There is no payload for this packet
            // Only resync the timer if the DotBot has already been registered.
            if (_tdma_vars.registration_flag == DB_TDMA_REGISTERED) {

                // update the state machine
                _tdma_vars.rx_flag = DB_TDMA_RX_WAIT;

                // Enable radio RX
                db_radio_rx();

                // Update the timer interrupts
                db_timer_hf_set_oneshot_us(TDMA_HF_TIMER_CC_TX, _tdma_vars.tdma_table.tx_start, &timer_tx_interrupt);
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
void timer_tmda_interrupt(void) {

    // TODO: implement this thing
    //  then implement what happens when you receive a radio packet.
    //  then implement what happens when someone registers
    //  .. when someone is lost
    //  when they transmit outside teir position
    // when someone uses tdma_tx() function
    // priority, and normal queue
    //

    // Check if this is your turn to transmit  (client = device id)
    // transmit everything you can.
    // Maybe make an allowance of -100us to execute the next bit of logic
    // go back to listening before leaving

    // Save the timestamp start of the current slot, to ensure accurate computation of the end of the slot
    _tdma_vars.slot_start_ts = db_timer_hf_now();

    // If there are not enough clients, make sure the the minimum frame time is respected, even if is just filled with empty slots.
    uint8_t last_slot;
    if (_tdma_vars.tdma_table.frame_duration_us > TDMA_SERVER_DEFAULT_FRAME_DURATION_US) {
        // enough clients, wrap around the end of the list
        last_slot = _tdma_vars.tdma_table.table_index;
    } else {
        // not enough clients, use the default amount of slots for the default frame duration
        last_slot = (TDMA_SERVER_DEFAULT_FRAME_DURATION_US / TDMA_SERVER_TIME_SLOT_DURATION_US) - 1;
    }
    // Update the active client for this slot. Wrap around after the end of the active clients is reached.
    _tdma_vars.active_slot_idx = (_tdma_vars.active_slot_idx + 1) % (last_slot);

    // Set the next interrupt
    // Implement if you ever use variable timeslots fr the dotbots
    // db_timer_hf_set_oneshot_us(TDMA_SERVER_HF_TIMER_CC, _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration, &timer_tdma_interrupt);  // start advertising behaviour

    // check if this is the start of the super frame to send a sync message.
    if (_tdma_vars.active_slot_idx == 0) {

        // Update last-superframe timestamp
        _tdma_vars.frame_start_ts = _tdma_vars.slot_start_ts;

        // Send a resync frame
        _send_sync_frame();
    }

    bool packet_sent = false;

    // Check that it's your timeslot
    if (_tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].client == _tdma_vars.device_id) {

        // TODO: Send registration messages. + Out of slot messages

        // send messages if available. time_available (slot_start + slot_duration - current_time)
        uint32_t remaining_slot_time_us = _tdma_vars.slot_start_ts + _tdma_vars.tdma_table.table[_tdma_vars.active_slot_idx].tx_duration - db_timer_hf_now();
        packet_sent                     = _send_queued_messages(remaining_slot_time_us - TDMA_TX_DEADTIME_US);

        // mark last time you sent anything
        if (packet_sent) {
            _tdma_vars.last_tx_packet_ts = db_timer_hf_now();
        }
    }
}
