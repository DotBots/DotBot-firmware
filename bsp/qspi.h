#ifndef __QSPI_H
#define __QSPI_H

/**
 * @defgroup    bsp_qspi QSPI flash
 * @ingroup     bsp
 * @brief       Control the QSPI flash peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2024-present
 * @}
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"

#define DB_QSPI_SCKFREQ_32MHZ (0)
#define DB_QSPI_SCKFREQ_16MHZ (1)
#define DB_QSPI_SCKFREQ_8MHZ  (3)
#define DB_QSPI_SCKFREQ_4MHZ  (7)
#define DB_QSPI_SCKFREQ_2MHZ  (15)
#define DB_QSPI_SCKFREQ_1MHZ  (31)

/// SPIM pin configuration
typedef struct {
    const gpio_t *io0;          ///< IO0
    const gpio_t *io1;          ///< IO1
    const gpio_t *io2;          ///< IO2
    const gpio_t *io3;          ///< IO3
    const gpio_t *cs;           ///< Chip select
    const gpio_t *sck;          ///< Clock
    uint8_t       sckfreq;      ///< QSPI clock frequency. 32MHz / (val + 1)
    bool          enable_quad;  ///< enable Quad serial mode
} db_qspi_conf_t;

/// SPIM mode
typedef enum {
    DB_SPIM_MODE_0 = 0,  ///< CPOL = 0 (Active High), CPHA = 0 (Leading)
    DB_SPIM_MODE_1,      ///< CPOL = 0 (Active High), CPHA = 1 (Trailing)
    DB_SPIM_MODE_2,      ///< CPOL = 1 (Active Low), CPHA = 0 (Leading)
    DB_SPIM_MODE_3       ///< CPOL = 1 (Active Low), CPHA = 1 (Trailing)
} db_qspi_mode_t;

/**
 * @brief Initialize the QSPI peripheral
 *
 * @param[in] conf      pointer to configuration struct
 */
void db_qspi_init(const db_qspi_conf_t *conf);

/**
 * @brief Read bytes from flash
 *
 * @param[in]   addr    24bit address
 * @param[in]   in      Pointer to the incoming bytes
 * @param[in]   len     Length of the bytes to send
 */
void db_qspi_read(const uint32_t addr, void *in, size_t len);

/**
 * @brief Program bytes on flash
 *
 * @param[in]   addr    24bit address
 * @param[in]   out     Pointer to the output bytes
 * @param[in]   len     Length of the bytes to receive
 */
void db_qspi_program(const uint32_t addr, const void *out, size_t len);

/**
 * @brief Erase page/blocks of flash
 *
 * @param[in]   addr    24bit address
 */
void db_qspi_block_erase(const uint32_t addr);

/**
 * @brief Erase all flash
 */
void db_qspi_bulk_erase(void);

#endif
