#ifndef __N25Q128_H
#define __N25Q128_H

/**
 * @defgroup    drv_n25q128     N25Q128 flash memory driver
 * @ingroup     drv
 * @brief       Driver for the Micron N25Q128 flash memory driver
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024-present
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

#include "gpio.h"

// Memory organization constants
#define N25Q128_PAGE_SIZE                         (256U)        ///< size in Bytes
#define N25Q128_PAGE_COUNT                        (0x00010000)  ///< 65536
#define N25Q128_SECTOR_SIZE                       (0x00010000)  ///< 64KB
#define N25Q128_MAIN_SECTOR_COUNT                 (248U)        ///< 248
#define N25Q125_BOOT_SECTOR_COUNT                 (8)           ///< 8 * 64KB
#define N25Q128_MAIN_MEMORY_BOTTOM_START_ADDRESS  (0x00080000)  ///< 3-bytes address
#define N25Q128_MAIN_MEMORY_TOP_START_ADDRESS     (0x00000000)  ///< 3-bytes address
#define N25Q128_MAIN_MEMORY_UNIFORM_START_ADDRESS (0x00000000)  ///< 3-bytes address
#define N25Q128_ADDRESS_LENGTH                    (3)           ///< 3-bytes length address

// Read identification constants
#define N25Q128_MANUFACTURER    (0x01)
#define N25Q128_MEMORY_TYPE     (0x20)
#define N25Q128_MEMORY_CAPACITY (0x18)  ///< 128Mbit
#define N25Q128_EDID_LENGTH     (0x10)

// EDID byte bitfields
#define N25Q128_EDID_VCR_XIP_MASK      (1 << 4)
#define N25Q128_EDID_HOLD_RESET_MASK   (1 << 3)
#define N25Q128_EDID_ADRESSING_MASK    (1 << 2)
#define N25Q128_EDID_ARCHITECTURE_MASK (0xFF)

/// N25Q128 architecture types
typedef enum {
    N25Q128_ARCHITECTURE_UNIFORM = 0x00,
    N25Q128_ARCHITECTURE_BOTTOM  = 0x01,
    N25Q128_ARCHITECTURE_TOP     = 0x11,
} n25q128_architecture_t;

/// Identification data
typedef struct __attribute__((packed)) {
    uint8_t manufacturer;  ///< Manufacturer
    uint8_t memory_type;   ///< Memory type
    uint8_t memory_size;   ///< Memory size
    uint8_t uid_length;    ///< Length of Extended device ID + Customized Factory Data
    uint8_t edid;          ///< Extended device ID
} n25q128_identification_t;

/// N25Q128 init configuration
typedef struct {
    const gpio_t *sck;   ///< SCK gpio
    const gpio_t *mosi;  ///< MOSI gpio
    const gpio_t *miso;  ///< MISO gpio
    const gpio_t *cs;    ///< CS gpio
} n25q128_conf_t;

/**
 * @brief Initializes the N25Q128 flash memory
 *
 * @param[in] conf          pointer to the init configuration
 */
void n25q128_init(const n25q128_conf_t *conf);

/**
 * @brief Returns the memory base address from the read identification
 *
 * @param[out] address      pointer to result base address
 */
void n25q128_base_address(uint32_t *address);

/**
 * @brief Reads status register
 *
 * @param[out] status       pointer to the output status byte
 */
void n25q128_read_status_register(uint8_t *status);

/**
 * @brief Write data in a page
 *
 * @param[in] address       address (only the 3 lower bytes are used)
 * @param[in] in            pointer to input array
 * @param[in] length        number of bytes to program
 */
void n25q128_program_page(uint32_t address, uint8_t *in, size_t length);

/**
 * @brief Erase a sector on the flash memory
 *
 * @param[in] address       address (only the 3 lower bytes are used)
 */
void n25q128_sector_erase(uint32_t address);

/**
 * @brief Enable flash write
 */
void n25q128_write_enable(void);

/**
 * @brief Disable flash write
 */
void n25q128_write_disable(void);

/**
 * @brief Erase whole flash memory
 */
void n25q128_bulk_erase(void);

/**
 * @brief Read data on flash memory
 *
 * @param[in] address       address (only the 3 lower bytes are used)
 * @param[out] out          pointer to the destination array
 * @param[in] length        number of bytes to read
 */
void n25q128_read(uint32_t address, uint8_t *out, size_t length);

#endif
