/**
 * @file
 * @defgroup project_freebot    FreeBot application
 * @ingroup projects
 * @brief This is the radio-controlled FreBot app
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "gpio.h"
#include "protocol.h"
#include "pwm.h"
#include "radio.h"
#include "timer.h"
// Include DRV headers
#include "tdma_client.h"

//=========================== defines ==========================================

#define TIMER_DEV                 (0)
#define DB_RADIO_FREQ             (8)      //< Set the frequency to 2408 MHz
#define DB_ADVERTIZEMENT_DELAY_MS (500U)   ///< 500ms delay between each advertizement packet sending
#define DB_TIMEOUT_CHECK_DELAY_MS (200U)   ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_BUFFER_MAX_BYTES       (255U)   ///< Max bytes in UART receive buffer
#define PWM_CHANNELS              (4)
#define M_TOP                     (100)

typedef struct {
    uint32_t ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    uint8_t  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    bool     advertize;                          ///< Whether an advertize packet should be sent
    uint64_t device_id;                          ///< Device ID of the DotBot
} freebot_vars_t;

//=========================== variables ========================================

static freebot_vars_t _freebot_vars;

static const gpio_t _freebot_front_motors_pins[] = {
    { .port = DB_MOTOR_AIN1_PORT, .pin = DB_MOTOR_AIN1_PIN },
    { .port = DB_MOTOR_BIN1_PORT, .pin = DB_MOTOR_BIN1_PIN },
    { .port = DB_MOTOR_AIN2_PORT, .pin = DB_MOTOR_AIN2_PIN },
    { .port = DB_MOTOR_BIN2_PORT, .pin = DB_MOTOR_BIN2_PIN },
};

static const gpio_t _freebot_back_motors_pins[] = {
    { .port = DB_MOTOR_AIN3_PORT, .pin = DB_MOTOR_AIN3_PIN },
    { .port = DB_MOTOR_BIN3_PORT, .pin = DB_MOTOR_BIN3_PIN },
    { .port = DB_MOTOR_AIN4_PORT, .pin = DB_MOTOR_AIN4_PIN },
    { .port = DB_MOTOR_BIN4_PORT, .pin = DB_MOTOR_BIN4_PIN },
};

static void _timeout_check(void);
static void _advertise(void);
void        _motors_init(void);
void        _motors_set(int16_t m1_speed, int16_t m2_speed, int16_t m3_speed, int16_t m4_speed);

//=========================== callbacks ========================================

static void _radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _freebot_vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);
    uint8_t           *ptk_ptr            = pkt;
    protocol_header_t *header             = (protocol_header_t *)ptk_ptr;
    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _freebot_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and update the motors' speeds
    switch ((uint8_t)*cmd_ptr++) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            const int16_t                m1      = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            const int16_t                m2      = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            const int16_t                m3      = m1;
            const int16_t                m4      = m2;
            _motors_set(m1, m2, m3, m4);
        } break;
        default:
            break;
    }
}

//=========================== main =============================================

int main(void) {
    db_tdma_client_init(&_radio_callback, DB_RADIO_BLE_1MBit, DB_RADIO_FREQ);

    // db_gpio_init(&db_led1, DB_GPIO_OUT);
    //  Retrieve the device id once at startup
    _freebot_vars.device_id = db_device_id();
    _freebot_vars.advertize = false;

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(TIMER_DEV, 1, DB_ADVERTIZEMENT_DELAY_MS, &_advertise);

    _motors_init();

    while (1) {
        __WFE();

        if (_freebot_vars.advertize) {
            // db_gpio_toggle(&db_led1);
            size_t length = db_protocol_advertizement_to_buffer(_freebot_vars.radio_buffer, DB_GATEWAY_ADDRESS, DotBot);
            db_tdma_client_tx(_freebot_vars.radio_buffer, length);
            _freebot_vars.advertize = false;
        }
    }
}

//=========================== private functions ================================

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks(TIMER_DEV);
    if (ticks > _freebot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        _motors_set(0, 0, 0, 0);
    }
}

static void _advertise(void) {
    _freebot_vars.advertize = true;
}

void _motors_init(void) {
    db_pwm_init(0, _freebot_front_motors_pins, PWM_CHANNELS, M_TOP);
    db_pwm_init(1, _freebot_back_motors_pins, PWM_CHANNELS, M_TOP);
}

void _motors_set(int16_t m1_speed, int16_t m2_speed, int16_t m3_speed, int16_t m4_speed) {
    // Double check for out-of-bound values.
    if (m1_speed > 100)
        m1_speed = 100;
    if (m1_speed < -100)
        m1_speed = -100;

    if (m2_speed > 100)
        m2_speed = 100;
    if (m2_speed < -100)
        m2_speed = -100;

    if (m3_speed > 100)
        m3_speed = 100;
    if (m3_speed < -100)
        m3_speed = -100;

    if (m4_speed > 100)
        m4_speed = 100;
    if (m4_speed < -100)
        m4_speed = -100;

    uint16_t front_pwm_seq[PWM_CHANNELS] = { 0 };
    uint16_t back_pwm_seq[PWM_CHANNELS]  = { 0 };

    if (m1_speed >= 0) {
        front_pwm_seq[0] = m1_speed;
        front_pwm_seq[1] = 0;
    }
    if (m1_speed < 0) {
        m1_speed *= -1;

        front_pwm_seq[0] = 0;
        front_pwm_seq[1] = m1_speed;
    }

    if (m2_speed >= 0) {
        front_pwm_seq[2] = m2_speed;
        front_pwm_seq[3] = 0;
    }
    if (m2_speed < 0) {
        m2_speed *= -1;

        front_pwm_seq[2] = 0;
        front_pwm_seq[3] = m2_speed;
    }

    if (m3_speed >= 0) {
        back_pwm_seq[0] = m3_speed;
        back_pwm_seq[1] = 0;
    }
    if (m3_speed < 0) {
        m3_speed *= -1;

        back_pwm_seq[0] = 0;
        back_pwm_seq[1] = m3_speed;
    }

    if (m4_speed >= 0) {
        back_pwm_seq[2] = m4_speed;
        back_pwm_seq[3] = 0;
    }
    if (m4_speed < 0) {
        m4_speed *= -1;

        back_pwm_seq[2] = 0;
        back_pwm_seq[3] = m4_speed;
    }

    // Update PWM values
    db_pwm_channels_set(0, front_pwm_seq);
    db_pwm_channels_set(1, back_pwm_seq);
}
