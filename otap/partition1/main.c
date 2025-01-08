/**
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application can be flashed on partition 1 (address depends on platform, see FLASH1 field in MemoryMap.xml)
 *
 * @copyright Inria, 2023
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <nrf.h>

#include "board_config.h"
#include "gpio.h"
#include "radio.h"

#include "ota.h"
#include "partition.h"
#ifdef DB_LED2_PIN
#include "timer.h"
#endif

//=========================== defines ==========================================

#define TIMER_DEV (0)

typedef struct {
    db_partitions_table_t table;
    db_ota_cpu_type_t     cpu;
    bool                  packet_received;
    uint8_t               message_buffer[UINT8_MAX];
} application_vars_t;

//=========================== variables ========================================

static application_vars_t _app_vars = { 0 };

//=========================== callbacks ========================================

static void _radio_callback(uint8_t *pkt, uint8_t len) {
    memcpy(&_app_vars.message_buffer, pkt, len);
    _app_vars.packet_received = true;
}

#if defined(DB_LED1_PIN) && defined(DB_BTN1_PIN)
static void _btn_toggle_callback(void *ctx) {
    (void)ctx;
    db_gpio_toggle(&db_led1);
}
#endif

#ifdef DB_LED2_PIN
static void _toggle_led(void) {
    db_gpio_toggle(&db_led2);
}
#endif

//=========================== private ==========================================

static void _ota_reply(const uint8_t *message, size_t len) {
    db_radio_disable();
    db_radio_tx(message, len);
}

static const db_ota_conf_t _ota_config = {
    .mode  = DB_OTA_MODE_DEFAULT,
    .reply = _ota_reply,
};

//================================ main ========================================

int main(void) {
    printf("Booting on partition %d (Build: %s)", DOTBOT_PARTITION, DOTBOT_BUILD_TIME);

    db_ota_init(&_ota_config);

    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);
    db_radio_rx();

#ifdef DB_LED1_PIN
    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_gpio_set(&db_led1);
#endif

#ifdef DB_LED2_PIN
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_set(&db_led2);
#endif

#if defined(DB_LED1_PIN) && defined(DB_BTN1_PIN)
    db_gpio_init_irq(&db_btn1, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, NULL);
#endif

#ifdef DB_LED2_PIN
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, 500, &_toggle_led);
#endif

    while (1) {
        __WFE();

        if (_app_vars.packet_received) {
            _app_vars.packet_received = false;
            db_ota_handle_message(_app_vars.message_buffer);
        }
    }
}
