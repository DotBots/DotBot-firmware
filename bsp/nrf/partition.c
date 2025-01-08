/**
 * @file
 * @ingroup bsp_partition
 *
 * @brief  Implementation of the "partition" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include <string.h>

#include <nrf.h>
#include "nvmc.h"
#include "partition.h"

//=========================== defines =========================================

//=========================== defines =========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_NVMC NRF_NVMC_NS
#elif defined(NRF_APPLICATION)
#define NRF_NVMC NRF_NVMC_S
#endif
#endif

#define DB_PARTITIONS_TABLE_ADDRESS (0x00001000UL + DB_FLASH_OFFSET)

//=========================== public ==========================================

void db_read_partitions_table(db_partitions_table_t *partitions) {
    memcpy((void *)partitions, (uint32_t *)DB_PARTITIONS_TABLE_ADDRESS, sizeof(db_partitions_table_t));
}

void db_write_partitions_table(const db_partitions_table_t *partitions) {
    uint32_t *addr   = (uint32_t *)DB_PARTITIONS_TABLE_ADDRESS;
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
#if defined(NRF5340_XXAA)
    *(uint32_t *)addr = 0xFFFFFFFF;
#else
    NRF_NVMC->ERASEPAGE = (uint32_t)addr;
#endif
    while (!NRF_NVMC->READY) {}

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    *addr++          = partitions->magic;
    *addr++          = partitions->length;
    *addr++          = partitions->active_image;
    for (uint32_t i = 0; i < partitions->length; i++) {
        *addr++ = partitions->partitions[i].address;
        *addr++ = partitions->partitions[i].size;
    }

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
}
