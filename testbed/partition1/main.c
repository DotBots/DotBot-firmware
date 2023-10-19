/**
 * @file main.c
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
#include "timer.h"

#include "ota.h"

//=========================== defines ==========================================

typedef struct {
    bool packet_received;
    db_ota_pkt_t ota_packet;
} application_vars_t;

//=========================== variables ========================================

static application_vars_t _app_vars = { 0 };

//=========================== callbacks ========================================

static void _radio_callback(uint8_t *pkt, uint8_t len) {
    if (len < sizeof(db_ota_pkt_t)) {
        printf("Invalid OTA packet (%d != %d)\n", len, sizeof(db_ota_pkt_t));
        return;
    }

    memcpy(&_app_vars.ota_packet, pkt, sizeof(db_ota_pkt_t));
    _app_vars.packet_received = true;
}

static void _btn_toggle_callback(void *ctx) {
    (void)ctx;
    db_gpio_toggle(&db_led1);
}

static void _toggle_led(void) {
    db_gpio_toggle(&db_led2);
}

//================================ main ========================================

int main(void) {
    printf("Booting on partition %d (Build: %s)", DOTBOT_PARTITION, DOTBOT_BUILD_TIME);

    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);
    db_radio_rx();

    db_gpio_init(&db_led1, DB_GPIO_OUT);
    db_gpio_set(&db_led1);
    db_gpio_init(&db_led2, DB_GPIO_OUT);
    db_gpio_set(&db_led2);
    db_gpio_init_irq(&db_btn1, DB_GPIO_IN_PU, DB_GPIO_IRQ_EDGE_RISING, _btn_toggle_callback, NULL);

    db_timer_init();
    db_timer_set_periodic_ms(0, 500, &_toggle_led);

    while (1) {
        __WFE();

        if (_app_vars.packet_received) {
            _app_vars.packet_received = false;
            if (_app_vars.ota_packet.index == 0) {
                puts("Starting firmware update");
                db_ota_start();
            }

            printf("Writing firmware chunk %u/%u\r", _app_vars.ota_packet.index, _app_vars.ota_packet.chunk_count);
            db_ota_write_chunk(&_app_vars.ota_packet);

            if (_app_vars.ota_packet.index == _app_vars.ota_packet.chunk_count - 1) {
                puts("Finalizing");
                db_ota_finish();
            }
        }
    }
}
