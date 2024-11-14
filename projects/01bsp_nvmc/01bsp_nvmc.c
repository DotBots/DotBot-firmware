/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the nvmc api.
 *
 * @copyright Inria, 2023
 *
 */

#include <nrf.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nvmc.h"

//=========================== defines ==========================================

#define PAGE_INDEX (DB_FLASH_PAGE_NUM / 2)            ///< Page in the middle of the flash
#define FLASH_ADDR (DB_FLASH_PAGE_SIZE * PAGE_INDEX)  ///< Address in flash corresponding to the page index

//=========================== variables ========================================

static const uint32_t *address    = (uint32_t *)(FLASH_ADDR + DB_FLASH_OFFSET);
static const char     *message    = "Hello DotBot";
static uint8_t         buffer[32] = { 0 };

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    // Whole page must be erased (written with 0xFF bytes) before new bytes can be written
    db_nvmc_page_erase(PAGE_INDEX);
    db_nvmc_write(address, message, strlen(message));
    db_nvmc_read(buffer, address, strlen(message));

    printf("Message written: %s (len: %d)\n", (char *)buffer, strlen(message));

    while (1) {
        __NOP();
    }
}
