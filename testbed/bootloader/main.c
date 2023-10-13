/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is a multi-purpose bootloader
 *
 * @copyright Inria, 2023
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "board_config.h"
#include "partition.h"
#include "gpio.h"
#include "nvmc.h"
#include "timer_hf.h"
#include "uart.h"

//=========================== defines =========================================

#define MAIN_IMAGE_ADDRESS       (0x00002000UL + DB_FLASH_OFFSET)
#define MAIN_IMAGE_SIZE          (0x00006FFFUL + DB_FLASH_OFFSET)  // 28K
#define USER_IMAGE_ADDRESS       (0x00008000UL + DB_FLASH_OFFSET)
#define USER_IMAGE_SIZE          (0x000F8000UL + DB_FLASH_OFFSET)  // 1M - 40K

#define DB_UART_MAX_BYTES (64U)
#define DB_UART_BAUDRATE  (115200U)

typedef struct {
    db_partitions_table_t table;
    uint8_t uart_buffer[DB_UART_MAX_BYTES];
    uint16_t uart_pos;
    uint32_t addr;
    bool write_page;
} bootloader_vars_t;

typedef struct {
    uint32_t msp;           ///< Main stack pointer
    uint32_t reset_handler; ///< Reset handler
} vector_table_t;

//=========================== variables ========================================

static const db_partitions_table_t bootstrap_table = {
    .magic = DB_PARTITIONS_TABLE_MAGIC,
    .length = 2,
    .active_image = 0,
    .partitions = {
        {
            .address = MAIN_IMAGE_ADDRESS,
            .size = MAIN_IMAGE_SIZE,
        },
        {
            .address = USER_IMAGE_ADDRESS,
            .size = USER_IMAGE_SIZE,
        },
    },
};

static bootloader_vars_t _bootloader_vars = { 0 };

//============================== private =======================================

static void _jump_to_image(uint32_t *address) {
    vector_table_t *table = (vector_table_t *)address; // Image should start with vector table
    // Set the vector table address prior to jumping to image
    SCB->VTOR = (uint32_t)table;
    __set_MSP(table->msp);      // Set main stack pointer
    __set_CONTROL(0x00000000);  // Clear control register
    __ISB();                    // Instruction sync
    ((void (*)(void))table->reset_handler)();  // Call reset handler of new image

    while (1) {}
}

static void _write_partitions_table(const db_partitions_table_t *table) {
    // Bootstrap the partition table and do a full reset
    db_write_partitions_table(table);
    NVIC_SystemReset();  // Reboot
}

static void _blink_led4(void) {
    db_gpio_toggle(&db_led4);
}

static void _uart_callback(uint8_t byte) {
    _bootloader_vars.uart_buffer[_bootloader_vars.uart_pos] = byte;
    _bootloader_vars.uart_pos++;
}

//================================= main =======================================

int main(void) {
    db_read_partitions_table(&_bootloader_vars.table);

    if (_bootloader_vars.table.magic != DB_PARTITIONS_TABLE_MAGIC) {
        // Bootstrap the partition table and do a system reset
        _write_partitions_table(&bootstrap_table);
        return 0;
    }

    uint32_t active_image = (_bootloader_vars.table.active_image < DB_PARTITIONS_MAX_COUNT) ? _bootloader_vars.table.active_image : 0;

    db_gpio_init(&db_btn4, DB_GPIO_IN_PU);
    uint8_t keep_active = !db_gpio_read(&db_btn4);

    if (keep_active) {
        db_gpio_init(&db_led4, DB_GPIO_OUT);
        db_gpio_init(&db_btn1, DB_GPIO_IN_PU);
        db_gpio_init(&db_btn2, DB_GPIO_IN_PU);
        db_timer_hf_init();
        db_timer_hf_set_periodic_us(0, 100000, &_blink_led4);
        db_uart_init(&db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_callback);
        _bootloader_vars.addr = _bootloader_vars.table.partitions[_bootloader_vars.table.active_image].address;
    }

    while (keep_active) {
        if (_bootloader_vars.uart_pos == DB_UART_MAX_BYTES) {
            _bootloader_vars.uart_pos = 0;
            if (_bootloader_vars.addr % DB_FLASH_PAGE_SIZE == 0) {
                db_nvmc_page_erase(_bootloader_vars.addr / DB_FLASH_PAGE_SIZE);
            }
            db_nvmc_write((uint32_t *)_bootloader_vars.addr, _bootloader_vars.uart_buffer, DB_UART_MAX_BYTES);
            _bootloader_vars.addr += DB_UART_MAX_BYTES;
        }

        if (!db_gpio_read(&db_btn1)) {
            _bootloader_vars.table.active_image = 0;
            _write_partitions_table(&_bootloader_vars.table);
        }

        if (!db_gpio_read(&db_btn2)) {
            _bootloader_vars.table.active_image = 1;
            _write_partitions_table(&_bootloader_vars.table);
        }
    }

     _jump_to_image((uint32_t *)_bootloader_vars.table.partitions[active_image].address);
}
