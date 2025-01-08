/**
 * @file
 * @ingroup drv_n25q128
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Module for driving the N25Q128 flash memory.
 *
 * @copyright Inria, 2024-present
 *
 */

#include <assert.h>
#include <stdio.h>

#include "gpio.h"
#include "spim.h"
#include "n25q128.h"

//=========================== defines ==========================================

#define SPIM_FREQUENCY DB_SPIM_FREQ_8M
#define SPIM_MODE      DB_SPIM_MODE_0

// Command set

// Reset operations
#define N25Q128_COMMAND_RESET_ENABLE (0x66)
#define N25Q128_COMMAND_RESET_MEMORY (0x99)

// Identification operations
#define N25Q128_COMMAND_READ_ID           (0x9F)
#define N25Q128_COMMAND_READ_SERIAL_FLASH (0x5A)

// Read operations
#define N25Q128_COMMAND_READ      (0x03)
#define N25Q128_COMMAND_FAST_READ (0x0B)

// Write operations
#define N25Q128_COMMAND_WRITE_ENABLE  (0x06)
#define N25Q128_COMMAND_WRITE_DISABLE (0x04)

// Register operations
#define N25Q128_COMMAND_READ_STATUS_REG               (0x05)
#define N25Q128_COMMAND_WRITE_STATUS_REG              (0x01)
#define N25Q128_COMMAND_READ_LOCK_REG                 (0xE8)
#define N25Q128_COMMAND_WRITE_LOCK_REG                (0xE5)
#define N25Q128_COMMAND_READ_FLAG_REG                 (0x70)
#define N25Q128_COMMAND_CLEAR_FLAG_REG                (0x50)
#define N25Q128_COMMAND_READ_NON_VOLATILE_CONFIG_REG  (0xB5)
#define N25Q128_COMMAND_WRITE_NON_VOLATILE_CONFIG_REG (0xB1)
#define N25Q128_COMMAND_READ_VOLATILE_CONFIG_REG      (0x85)
#define N25Q128_COMMAND_WRITE_VOLATILE_CONFIG_REG     (0x81)
#define N25Q128_COMMAND_READ_ENH_VOLATILE_CONFIG_REG  (0x65)
#define N25Q128_COMMAND_WRITE_ENH_VOLATILE_CONFIG_REG (0x61)

// Program operations
#define N25Q128_COMMAND_PAGE_PROGRAM (0x02)

// Erase operations
#define N25Q128_COMMAND_SUBSECTOR_ERASE (0x20)
#define N25Q128_COMMAND_SECTOR_ERASE    (0xD8)
#define N25Q128_COMMAND_BULK_ERASE      (0xC7)
#define N25Q128_COMMAND_ERASE_RESUME    (0x7A)
#define N25Q128_COMMAND_ERASE_SUSPEND   (0x75)

#define SPIM_DEV (0)

typedef struct {
    gpio_t                   cs;
    n25q128_identification_t id;
} n25q128_vars_t;

//=========================== variables ========================================

static n25q128_vars_t _n25q128_vars = { 0 };

//============================== private =======================================

static void _send_command(uint8_t command) {
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, sizeof(uint8_t));
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
}

static void _read_identification(void) {
    uint8_t command = N25Q128_COMMAND_READ_ID;
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, 1);
    db_spim_receive(SPIM_DEV, &_n25q128_vars.id, sizeof(n25q128_identification_t));
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
}

//============================== public ========================================

void n25q128_init(const n25q128_conf_t *conf) {
    const db_spim_conf_t _spim_conf = {
        .mosi = conf->mosi,
        .sck  = conf->sck,
        .miso = conf->miso,
    };
    _n25q128_vars.cs.port = conf->cs->port;
    _n25q128_vars.cs.pin  = conf->cs->pin;

    db_spim_init(SPIM_DEV, &_spim_conf);

    db_gpio_init(&_n25q128_vars.cs, DB_GPIO_OUT);
    db_gpio_set(&_n25q128_vars.cs);

    _read_identification();
    assert(_n25q128_vars.id.manufacturer == N25Q128_MANUFACTURER);
    assert(_n25q128_vars.id.memory_type == N25Q128_MEMORY_TYPE);
    assert(_n25q128_vars.id.memory_size == N25Q128_MEMORY_CAPACITY);
}

void n25q128_base_address(uint32_t *address) {
    if ((_n25q128_vars.id.edid & N25Q128_EDID_ARCHITECTURE_MASK) == N25Q128_ARCHITECTURE_BOTTOM) {
        *address = N25Q128_MAIN_MEMORY_BOTTOM_START_ADDRESS;
        return;
    }
    *address = N25Q128_MAIN_MEMORY_UNIFORM_START_ADDRESS;
}

void n25q128_reset(void) {
    _send_command(N25Q128_COMMAND_RESET_ENABLE);
}

void n25q128_write_enable(void) {
    _send_command(N25Q128_COMMAND_WRITE_ENABLE);
    uint8_t status = 0;
    do {
        n25q128_read_status_register(&status);
    } while (!(status & 0b00000010));
}

void n25q128_write_disable(void) {
    _send_command(N25Q128_COMMAND_WRITE_DISABLE);
    uint8_t status = 0;
    do {
        n25q128_read_status_register(&status);
    } while (status & 0b00000010);
}

void n25q128_read_status_register(uint8_t *status) {
    uint8_t command = N25Q128_COMMAND_READ_STATUS_REG;
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, sizeof(uint8_t));
    db_spim_receive(SPIM_DEV, status, sizeof(uint8_t));
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
}

void n25q128_program_page(uint32_t address, uint8_t *in, size_t length) {
    n25q128_write_enable();

    uint8_t  command[4];
    uint8_t *addr_ptr = (uint8_t *)&address;
    command[0]        = N25Q128_COMMAND_PAGE_PROGRAM;
    command[1]        = addr_ptr[2];
    command[2]        = addr_ptr[1];
    command[3]        = addr_ptr[0];
    // memcpy(&command[1], &address, 3);
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, 4);
    db_spim_send(SPIM_DEV, in, length);
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
    uint8_t status = 0;
    do {
        n25q128_read_status_register(&status);
    } while (status & 0b00000001);
}

void n25q128_sector_erase(uint32_t address) {
    n25q128_write_enable();

    uint8_t  command[4];
    uint8_t *addr_ptr = (uint8_t *)&address;
    command[0]        = N25Q128_COMMAND_SECTOR_ERASE;
    command[1]        = addr_ptr[2];
    command[2]        = addr_ptr[1];
    command[3]        = addr_ptr[0];
    // memcpy(&command[1], &address, 3);
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, 4);
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
    uint8_t status = 0;
    do {
        n25q128_read_status_register(&status);
    } while (status & 0b00000001);
}

void n25q128_bulk_erase(void) {
    n25q128_write_enable();

    _send_command(N25Q128_COMMAND_BULK_ERASE);
    uint8_t status = 0;
    do {
        n25q128_read_status_register(&status);
    } while (status & 0b00000001);
}

void n25q128_read(uint32_t address, uint8_t *out, size_t length) {
    uint8_t  command[4];
    uint8_t *addr_ptr = (uint8_t *)&address;
    command[0]        = N25Q128_COMMAND_READ;
    command[1]        = addr_ptr[2];
    command[2]        = addr_ptr[1];
    command[3]        = addr_ptr[0];
    db_spim_begin(SPIM_DEV, &_n25q128_vars.cs, SPIM_MODE, SPIM_FREQUENCY);
    db_spim_send(SPIM_DEV, &command, 4);
    db_spim_receive(SPIM_DEV, out, length);
    db_spim_end(SPIM_DEV, &_n25q128_vars.cs);
}
