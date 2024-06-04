/**
 * @file
 * @defgroup project_lh2_mini_mote    LH2-minimote test app
 * @ingroup projects
 * @brief This is the radio-controlled LH2 mini mote app
 *
 * Test every functionality of the minimote, to check it works.
 * IMU orientation changes the color of the RGB led
 * The LED flashed white when it receives an LH2 packet.
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
#include "timer.h"
// Include DRV headers
#include "ism330.h"

//=========================== defines ==========================================

#define DB_LH2_UPDATE_DELAY_US    (100000U)  ///< 100ms delay between each LH2 data refresh
#define DB_ADVERTIZEMENT_DELAY_US (500000U)  ///< 500ms delay between each advertizement packet sending
#define IMU_COLOR_DELAY_US        (100000U)  ///< 100ms delay between each IMU color update
#define DB_BUFFER_MAX_BYTES       (255U)     ///< Max bytes in UART receive buffer

typedef enum {
    BLACK,
    RED,
    GREEN,
    BLUE,
} led_color_t;

typedef struct {
    uint32_t    ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t    lh2;                                ///< LH2 device descriptor
    uint8_t     radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    bool        advertize;                          ///< Whether an advertize packet should be sent
    bool        update_color_flag;                  ///< Flag - Time to read the IMU and update the color of the LED
    uint64_t    device_id;                          ///< Device ID of the DotBot
    led_color_t color;                              ///< current color of the RBG defined by the IMU
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;
static const gpio_t  _r_led_pin = { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN };
static const gpio_t  _g_led_pin = { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN };
static const gpio_t  _b_led_pin = { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN };

//=========================== prototypes =======================================

static void _advertise(void);
static void _previous_led_color(void);
static void radio_callback(uint8_t *pkt, uint8_t len);
static void _imu_to_rgb(void);
static void _set_rgb_led(void);
static void _update_color(void);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_ism330_init(&db_sda, &db_scl);
    db_protocol_init();
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx();              // Start receiving packets.

    // Set an invalid heading since the value is unknown on startup.
    // Control loop is stopped and advertize packets are sent
    _dotbot_vars.advertize = false;

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    // Setup up timer interrupts
    db_timer_hf_init(0);
    db_timer_hf_set_periodic_us(0, 1, DB_ADVERTIZEMENT_DELAY_US, &_advertise);
    db_timer_hf_set_periodic_us(0, 2, IMU_COLOR_DELAY_US, &_update_color);
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
                    db_gpio_clear(&_b_led_pin);
                    db_gpio_clear(&_g_led_pin);
                    db_timer_hf_set_oneshot_us(0, 0, 3000, &_previous_led_color);

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
                    db_radio_disable();
                    db_radio_tx(_dotbot_vars.radio_buffer, length);

                    // Mark the data as already sent
                    _dotbot_vars.lh2.data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
                }
            }
        }

        if (_dotbot_vars.update_color_flag) {
            // Check the IMU and change the color of the RBG LED
            _imu_to_rgb();
            _set_rgb_led();
            _dotbot_vars.update_color_flag = false;
        }

        if (_dotbot_vars.advertize) {
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
            size_t length = sizeof(protocol_header_t);
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.advertize = false;
        }
    }
}

//=========================== private functions ================================

static void _advertise(void) {
    _dotbot_vars.advertize = true;
}

static void _update_color(void) {
    _dotbot_vars.update_color_flag = true;
}

static void _previous_led_color(void) {
    _set_rgb_led();
}

static void _imu_to_rgb(void) {
    // read IMU
    static ism330_acc_data_t acc_data;
    db_ism330_accel_read(&acc_data);

    // Decide the color of the LED
    // Calculate the absolute value.
    if (acc_data.x < 0)
        acc_data.x = acc_data.x * -1;
    if (acc_data.y < 0)
        acc_data.y = acc_data.y * -1;
    if (acc_data.z < 0)
        acc_data.z = acc_data.z * -1;

    // Ligth an LED according to the direction of gravity.
    if ((acc_data.x > acc_data.y) && (acc_data.x > acc_data.z)) {
        _dotbot_vars.color = RED;
    } else if ((acc_data.y > acc_data.x) && (acc_data.y > acc_data.z)) {
        _dotbot_vars.color = GREEN;
    } else if ((acc_data.z > acc_data.x) && (acc_data.z > acc_data.y)) {
        _dotbot_vars.color = BLUE;
    } else {
        _dotbot_vars.color = BLACK;
    }
}

static void _set_rgb_led(void) {
    switch (_dotbot_vars.color) {
        case RED:
            db_gpio_clear(&_r_led_pin);
            db_gpio_set(&_g_led_pin);
            db_gpio_set(&_b_led_pin);
            break;

        case GREEN:
            db_gpio_set(&_r_led_pin);
            db_gpio_clear(&_g_led_pin);
            db_gpio_set(&_b_led_pin);
            break;

        case BLUE:
            db_gpio_set(&_r_led_pin);
            db_gpio_set(&_g_led_pin);
            db_gpio_clear(&_b_led_pin);
            break;

        case BLACK:
            db_gpio_set(&_r_led_pin);
            db_gpio_set(&_g_led_pin);
            db_gpio_set(&_b_led_pin);
            break;

        default:
            db_gpio_set(&_r_led_pin);
            db_gpio_set(&_g_led_pin);
            db_gpio_set(&_b_led_pin);
            break;
    }
}

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_ticks(0);
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
