/**
 * @file
 * @defgroup project_lh2_mini_mote    LH2-minimote application
 * @ingroup projects
 * @brief This is the radio-controlled LH2 mini mote app
 *
 * Control the RGB LED of the minimote, and receive the LH2 packets
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "radio.h"
#include "rgbled_pwm.h"
#include "timer_hf.h"
// Include DRV headers
#include "frame.h"

//=========================== defines ==========================================

#define TIMER_DEV                 (0)
#define DB_LH2_UPDATE_DELAY_US    (100000U)  ///< 100ms delay between each LH2 data refresh
#define DB_ADVERTISEMENT_DELAY_US (500000U)  ///< 500ms delay between each advertizement packet sending
#define DB_BUFFER_MAX_BYTES       (255U)     ///< Max bytes in UART receive buffer

typedef struct {
    uint32_t ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t lh2;                                ///< LH2 device descriptor
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    uint64_t device_id;                          ///< Device ID of the LH2 mini mote
    bool     advertise;                          ///< Whether an advertize packet should be sent
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;
static const gpio_t  _r_led_pin = { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN };

//=========================== prototypes =======================================

static void _advertise(void);
static void _turn_off_led(void);
static void radio_callback(uint8_t *pkt, uint8_t len);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    _dotbot_vars.advertise = false;
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_network_address(DB_FRAME_ACCESS_ADDR);
    db_radio_set_frequency(DB_FRAME_DEFAULT_FREQ);
    db_radio_rx();

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    // Setup up timer interrupts
    db_timer_hf_init(TIMER_DEV);
    db_timer_hf_set_periodic_us(TIMER_DEV, 1, DB_ADVERTISEMENT_DELAY_US, &_advertise);
    db_lh2_init(&_dotbot_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    while (1) {
        __WFE();

        // Process available lighthouse data
        db_lh2_process_location(&_dotbot_vars.lh2);

        // Check if data is ready to send
        for (size_t basestation = 0; basestation < 2; basestation++) {
            for (size_t sweep = 0; sweep < 2; sweep++) {
                if (_dotbot_vars.lh2.data_ready[sweep][basestation] == DB_LH2_PROCESSED_DATA_AVAILABLE) {

                    db_gpio_clear(&_r_led_pin);
                    db_timer_hf_set_oneshot_us(TIMER_DEV, 0, 3000, &_turn_off_led);

                    // Prepare the radio buffer
                    size_t length = db_frame_header_to_buffer(_dotbot_vars.radio_buffer, DB_GATEWAY_ADDRESS);

                    // Package data into a variable
                    protocol_lh2_processed_packet_t lh2_packet;
                    lh2_packet.selected_polynomial = _dotbot_vars.lh2.locations[sweep][basestation].selected_polynomial;
                    lh2_packet.lfsr_counts         = _dotbot_vars.lh2.locations[sweep][basestation].lfsr_counts;
                    lh2_packet.timestamp_us        = _dotbot_vars.lh2.timestamps[sweep][basestation];

                    // Add the LH2 sweep
                    _dotbot_vars.radio_buffer[length++] = DB_PROTOCOL_LH2_PROCESSED_DATA;
                    memcpy(_dotbot_vars.radio_buffer + length, &lh2_packet, sizeof(protocol_lh2_processed_packet_t));
                    length += sizeof(protocol_lh2_processed_packet_t);

                    // Send through radio
                    db_radio_disable();
                    db_radio_tx(_dotbot_vars.radio_buffer, length);
                    db_radio_rx();

                    // Mark the data as already sent
                    _dotbot_vars.lh2.data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
                }
            }
        }

        if (_dotbot_vars.advertise) {
            size_t length                       = db_frame_header_to_buffer(_dotbot_vars.radio_buffer, DB_GATEWAY_ADDRESS);
            _dotbot_vars.radio_buffer[length++] = DB_PROTOCOL_ADVERTISEMENT;
            _dotbot_vars.radio_buffer[length++] = LH2_mini_mote;
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            db_radio_rx();
            _dotbot_vars.advertise = false;
        }
    }
}

//=========================== private functions ================================

static void _advertise(void) {
    _dotbot_vars.advertise = true;
}

static void _turn_off_led(void) {
    db_gpio_set(&_r_led_pin);
}

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    if (len < sizeof(db_frame_header_t) + 1) {
        return;
    }

    _dotbot_vars.ts_last_packet_received = db_timer_hf_now(TIMER_DEV);
    const db_frame_header_t *header      = (const db_frame_header_t *)pkt;
    // Drop frames addressed elsewhere
    if (header->dst != DB_FRAME_DST_BROADCAST && header->dst != _dotbot_vars.device_id) {
        return;
    }

    // Drop wrong version / wrong upper-layer protocol
    if (header->version != DB_FRAME_VERSION ||
        header->type != DB_FRAME_TYPE_DATA ||
        header->next_proto != DB_FRAME_NEXT_PROTO) {
        return;
    }

    uint8_t *cmd_ptr = pkt + sizeof(db_frame_header_t);
    // parse received packet and update the motors' speeds
    switch ((uint8_t)*cmd_ptr++) {

        case DB_PROTOCOL_CMD_RGB_LED:
        {
            const protocol_rgbled_command_t *command = (const protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
        } break;

        default:
            break;
    }
}
