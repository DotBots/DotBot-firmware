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
#include "tdma_client.h"

//=========================== defines ==========================================

#define TIMER_DEV              (0)
#define DB_RADIO_FREQ          (28)             //< Set the frequency to 2408 MHz
#define RADIO_APP              (LH2_mini_mote)  // LH2 mini mote Radio App
#define DB_LH2_UPDATE_DELAY_US (100000U)        ///< 100ms delay between each LH2 data refresh
#define DB_BUFFER_MAX_BYTES    (255U)           ///< Max bytes in UART receive buffer

typedef struct {
    uint32_t ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t lh2;                                ///< LH2 device descriptor
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    uint64_t device_id;                          ///< Device ID of the LH2 mini mote
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;
static const gpio_t  _r_led_pin = { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN };

//=========================== prototypes =======================================

static void _turn_off_led(void);
static void radio_callback(uint8_t *pkt, uint8_t len);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_protocol_init();
    db_tdma_client_init(&radio_callback, DB_RADIO_BLE_1MBit, DB_RADIO_FREQ, RADIO_APP);

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    // Setup up timer interrupts
    db_timer_hf_init(TIMER_DEV);
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
                    db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, LH2_mini_mote, DB_PROTOCOL_LH2_PROCESSED_DATA);

                    // Package data into a variable
                    protocol_lh2_processed_packet_t lh2_packet;
                    lh2_packet.selected_polynomial = _dotbot_vars.lh2.locations[sweep][basestation].selected_polynomial;
                    lh2_packet.lfsr_location       = _dotbot_vars.lh2.locations[sweep][basestation].lfsr_location;
                    lh2_packet.timestamp_us        = _dotbot_vars.lh2.timestamps[sweep][basestation];

                    // Add the LH2 sweep
                    memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), &lh2_packet, sizeof(protocol_lh2_processed_packet_t));
                    size_t length = sizeof(protocol_header_t) + sizeof(protocol_lh2_processed_packet_t);

                    // Send through radio
                    db_tdma_client_tx(_dotbot_vars.radio_buffer, length);

                    // Mark the data as already sent
                    _dotbot_vars.lh2.data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
                }
            }
        }
    }
}

//=========================== private functions ================================

static void _turn_off_led(void) {
    db_gpio_set(&_r_led_pin);
}

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_hf_now(TIMER_DEV);
    uint8_t           *ptk_ptr           = pkt;
    protocol_header_t *header            = (protocol_header_t *)ptk_ptr;
    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _dotbot_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Check application is compatible
    if (header->application != DotBot) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and update the motors' speeds
    switch (header->type) {

        case DB_PROTOCOL_CMD_RGB_LED:
        {
            protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
        } break;

        default:
            break;
    }
}
