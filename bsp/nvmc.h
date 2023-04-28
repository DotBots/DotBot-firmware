#ifndef __NVMC_H
#define __NVMC_H

/**
 * @file nvmc.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "nvmc" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdlib.h>
#include <stdint.h>

//=========================== defines =========================================

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define DB_FLASH_PAGE_SIZE 2048
#else
#define DB_FLASH_PAGE_SIZE 4096
#endif

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define DB_FLASH_OFFSET 0x01000000
#else
#define DB_FLASH_OFFSET 0x0
#endif

#if defined(NRF52840_XXAA)
#define DB_FLASH_PAGE_NUM 256
#elif defined(NRF52833_XXAA)
#define DB_FLASH_PAGE_NUM 128
#elif defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define DB_FLASH_PAGE_NUM 256
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define DB_FLASH_PAGE_NUM 128
#else
#error "Unsupported nRF CPU"
#endif

//=========================== public ==========================================

/**
 * @brief Read some data from a given address
 * @param data  Pointer to the output buffer
 * @param addr  Address to read from
 * @param len   Length of data to read
 */
void db_nvmc_read(void *output, const uint32_t *addr, size_t len);

/**
 * @brief Erase a page on flash
 *
 * @param page  index of the page to erase
 */
void db_nvmc_page_erase(uint32_t page);

/**
 * @brief Write some data at a given address
 *
 * @param addr  Address to write to
 * @param input Pointer to the intput buffer to write
 * @param len   Length of data to write
 */
void db_nvmc_write(const uint32_t *addr, const void *input, size_t len);

#endif
