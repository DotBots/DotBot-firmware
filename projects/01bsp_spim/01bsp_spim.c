/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the SPIM api.
 *
 * @copyright Inria, 2024-present
 *
 */
#include <nrf.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
#include "spim.h"

//============================ defines =========================================

#define SX127X_REG_OPMODE       (0x01)
#define SX1276_REG_VERSION      (0x42)
#define SX1276_VERSION_EXPECTED (0x12)

#define SPIM_DEV       (0)
#define SPIM_FREQUENCY DB_SPIM_FREQ_4M

//=========================== variables ========================================

#if defined(BOARD_NRF52833DK)
static const gpio_t _sck_pin  = { .port = 0, .pin = 23 };
static const gpio_t _miso_pin = { .port = 0, .pin = 22 };
static const gpio_t _mosi_pin = { .port = 0, .pin = 21 };
static const gpio_t _cs_pin   = { .port = 0, .pin = 20 };
#else
static const gpio_t _sck_pin  = { .port = 1, .pin = 15 };
static const gpio_t _miso_pin = { .port = 1, .pin = 14 };
static const gpio_t _mosi_pin = { .port = 1, .pin = 13 };
static const gpio_t _cs_pin   = { .port = 1, .pin = 12 };
#endif

const db_spim_conf_t _spim_conf = {
    .mosi = &_mosi_pin,
    .sck  = &_sck_pin,
    .miso = &_miso_pin,
};

static void _read_reg(uint8_t reg, uint8_t *value) {
    db_spim_begin(SPIM_DEV, &_cs_pin, DB_SPIM_MODE_0, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &reg, 1);
    db_spim_receive(SPIM_DEV, value, 1);
    db_spim_end(SPIM_DEV, &_cs_pin);
}

static void _write_reg(uint8_t reg, uint8_t value) {
    db_spim_begin(SPIM_DEV, &_cs_pin, DB_SPIM_MODE_0, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &reg, 1);
    db_spim_receive(SPIM_DEV, &value, 1);
    db_spim_end(SPIM_DEV, &_cs_pin);
}

//=========================== main =============================================

int main(void) {
    db_spim_init(SPIM_DEV, &_spim_conf);

    db_gpio_init(&_cs_pin, DB_GPIO_OUT);
    db_gpio_set(&_cs_pin);

    uint8_t version_addr = SX1276_REG_VERSION;
    uint8_t version      = 0;
    _read_reg(version_addr, &version);
    if (version != SX1276_VERSION_EXPECTED) {
        printf("[ERROR] Invalid SX1276 version: %d (expected: %d)\n", version, SX1276_VERSION_EXPECTED);
        assert(false);
    }

    uint8_t op_mode      = 0;
    uint8_t op_mode_addr = SX127X_REG_OPMODE;  // Sleep mode
    _write_reg(op_mode_addr, op_mode);

    puts("Success!");
    while (1) {
        __WFE();
    }
}
