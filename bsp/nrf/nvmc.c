/**
 * @file
 * @ingroup bsp_nvmc
 *
 * @brief  Implementation of the "nvmc" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <nrf.h>

#include "nvmc.h"

//=========================== defines =========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_NVMC NRF_NVMC_NS
#elif defined(NRF_APPLICATION)
#define NRF_NVMC NRF_NVMC_S
#endif
#endif

//=========================== public ==========================================

void db_nvmc_read(void *output, const uint32_t *addr, size_t len) {

    memcpy(output, addr, len);
}

void db_nvmc_page_erase(uint32_t page) {

    assert(page < DB_FLASH_PAGE_NUM);

    const uint32_t *addr = (const uint32_t *)(page * DB_FLASH_PAGE_SIZE + DB_FLASH_OFFSET);

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
#if defined(NRF5340_XXAA)
    *(uint32_t *)addr = 0xFFFFFFFF;
#else
    NRF_NVMC->ERASEPAGE = (uint32_t)addr;
#endif

    while (!NRF_NVMC->READY) {}
}

void db_nvmc_write(const uint32_t *addr, const void *data, size_t len) {

    // Length must be a multiple of 4 bytes
    assert(len % 4 == 0);
    // writes must be 4 bytes aligned
    assert(((uint32_t)addr % 4) && ((uint32_t)data % 4));

    uint32_t       *dest_addr = (uint32_t *)addr;
    const uint32_t *data_addr = data;

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    for (uint32_t i = 0; i < (len >> 2); i++) {
        *dest_addr++ = data_addr[i];
    }

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
}
