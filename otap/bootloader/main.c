/**
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is a multi-purpose bootloader
 *
 * @copyright Inria, 2023
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "board_config.h"
#include "partition.h"
#include "gpio.h"
#include "hdlc.h"
#include "nvmc.h"
#include "ota.h"
#include "timer_hf.h"
#include "uart.h"

//=========================== defines =========================================

#if defined(NRF52833_XXAA)
#define SLOT0_IMAGE_ADDRESS (0x00002000UL)
#define SLOT0_IMAGE_SIZE    (0x0003F000UL)  // 252K
#define SLOT1_IMAGE_ADDRESS (0x00041000UL)
#define SLOT1_IMAGE_SIZE    (0x0003F000UL)  // 252K
#elif defined(NRF5340_XXAA_NETWORK)
#define SLOT0_IMAGE_ADDRESS (0x01002000UL + DB_FLASH_OFFSET)
#define SLOT0_IMAGE_SIZE    (0x0001F000UL)  // 124K
#define SLOT1_IMAGE_ADDRESS (0x01021000UL + DB_FLASH_OFFSET)
#define SLOT1_IMAGE_SIZE    (0x0001F000UL)  // 124K
#else
#define SLOT0_IMAGE_ADDRESS (0x00002000UL)
#define SLOT0_IMAGE_SIZE    (0x0007F000UL)  // 508K
#define SLOT1_IMAGE_ADDRESS (0x00081000UL)
#define SLOT1_IMAGE_SIZE    (0x0007F000UL)  // 508K
#endif

#define DB_UART_MAX_BYTES (64U)
#define DB_UART_BAUDRATE  (1000000U)

#define DB_TIMER_DEV (0)  // Timer device index

typedef struct {
    db_partitions_table_t table;
    uint8_t               uart_byte;
    bool                  uart_byte_received;
    uint8_t               hdlc_buffer[UINT8_MAX];
} bootloader_vars_t;

typedef struct {
    uint32_t msp;            ///< Main stack pointer
    uint32_t reset_handler;  ///< Reset handler
} vector_table_t;

//=========================== variables ========================================

// clang-format off
static const db_partitions_table_t bootstrap_table = {
    .magic        = DB_PARTITIONS_TABLE_MAGIC,
    .length       = 2,
    .active_image = 0,
    .partitions   = {
        {
              .address = SLOT0_IMAGE_ADDRESS,
              .size    = SLOT0_IMAGE_SIZE,
        },
        {
              .address = SLOT1_IMAGE_ADDRESS,
              .size    = SLOT1_IMAGE_SIZE,
        },
    },
};
// clang-format on

static bootloader_vars_t _bootloader_vars = { 0 };

//============================== private =======================================

static void _jump_to_image(uint32_t *address) {
    vector_table_t *table = (vector_table_t *)address;  // Image should start with vector table
    // Set the vector table address prior to jumping to image
    SCB->VTOR = (uint32_t)table;
    __set_MSP(table->msp);                     // Set main stack pointer
    __set_CONTROL(0x00000000);                 // Clear control register
    __ISB();                                   // Instruction sync
    ((void (*)(void))table->reset_handler)();  // Call reset handler of new image

    while (1) {}
}

static void _write_partitions_table(const db_partitions_table_t *table) {
    // Bootstrap the partition table and do a full reset
    db_write_partitions_table(table);
    NVIC_SystemReset();  // Reboot
}

//=========================== callbacks ========================================

#ifdef DB_LED4_PIN
static void _blink_led4(void) {
    db_gpio_toggle(&db_led4);
}
#endif

#ifdef DB_BTN4_PIN
static void _uart_callback(uint8_t byte) {
    _bootloader_vars.uart_byte          = byte;
    _bootloader_vars.uart_byte_received = true;
}

//=========================== private ==========================================

static void _bootloader_reply(const uint8_t *message, size_t len) {
    size_t frame_len = db_hdlc_encode(message, len, _bootloader_vars.hdlc_buffer);
    db_uart_write(0, _bootloader_vars.hdlc_buffer, frame_len);
}

static const db_ota_conf_t _bootloader_ota_config = {
    .mode  = DB_OTA_MODE_BOOTLOADER,
    .reply = _bootloader_reply,
};
#endif

//================================= main =======================================

int main(void) {
    db_read_partitions_table(&_bootloader_vars.table);

    if (_bootloader_vars.table.magic != DB_PARTITIONS_TABLE_MAGIC) {
        // Bootstrap the partition table and do a system reset
        _write_partitions_table(&bootstrap_table);
        return 0;
    }

    uint32_t active_image = (_bootloader_vars.table.active_image < DB_PARTITIONS_MAX_COUNT) ? _bootloader_vars.table.active_image : 0;

#ifdef DB_BTN4_PIN
    db_gpio_init(&db_btn4, DB_GPIO_IN_PU);

    uint8_t keep_active = !db_gpio_read(&db_btn4);

    if (keep_active) {
        db_gpio_init(&db_led4, DB_GPIO_OUT);
        db_gpio_init(&db_btn1, DB_GPIO_IN_PU);
        db_gpio_init(&db_btn2, DB_GPIO_IN_PU);
        db_timer_hf_init(DB_TIMER_DEV);
        db_timer_hf_set_periodic_us(DB_TIMER_DEV, 0, 100000, &_blink_led4);
        db_uart_init(0, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_callback);
        db_ota_init(&_bootloader_ota_config);
    }
#else
    uint8_t keep_active = 0;
#endif

    while (keep_active) {
        if (_bootloader_vars.uart_byte_received) {
            _bootloader_vars.uart_byte_received = false;
            db_hdlc_state_t hdlc_state          = db_hdlc_rx_byte(_bootloader_vars.uart_byte);
            switch ((uint8_t)hdlc_state) {
                case DB_HDLC_STATE_IDLE:
                case DB_HDLC_STATE_RECEIVING:
                case DB_HDLC_STATE_ERROR:
                    break;
                case DB_HDLC_STATE_READY:
                {
                    size_t msg_len = db_hdlc_decode(_bootloader_vars.hdlc_buffer);
                    if (msg_len) {
                        db_ota_handle_message(_bootloader_vars.hdlc_buffer);
                    }
                } break;
                default:
                    break;
            }
        }

#ifdef DB_BTN1_PIN
        if (!db_gpio_read(&db_btn1)) {
            _bootloader_vars.table.active_image = 0;
            _write_partitions_table(&_bootloader_vars.table);
        }
#endif
#ifdef DB_BTN2_PIN
        if (!db_gpio_read(&db_btn2)) {
            _bootloader_vars.table.active_image = 1;
            _write_partitions_table(&_bootloader_vars.table);
        }
#endif
    }

    _jump_to_image((uint32_t *)_bootloader_vars.table.partitions[active_image].address);
}
