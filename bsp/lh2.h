#ifndef LH2_H_
#define LH2_H_

/**
 * @file lh2.h
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "lh2" bsp module.
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>, Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"

//=========================== defines ==========================================

#define LH2_LOCATIONS_COUNT 4  ///< Number of computed locations

typedef enum {
    DB_LH2_RUNNING,  ///< the lh2 engine is running
    DB_LH2_READY,    ///< some lh2 location is ready to be read
} db_lh2_state_t;

typedef struct {
    uint8_t  selected_polynomial;  ///< selected polynomial
    uint32_t lfsr_location;        ///< lfsr location
} db_lh2_result_t;

typedef struct {
    db_lh2_state_t  state;                         ///< current state of the lh2 engine
    db_lh2_result_t results[LH2_LOCATIONS_COUNT];  ///< buffer holding the location data
} db_lh2_t;

//=========================== public ===========================================

/**
 * @brief Initialize LH2
 *
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void db_lh2_init(const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief Compute the location based on available frames
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_process_location(db_lh2_t *lh2);

/**
 * @brief Reset the lh2 internal state so new location computation can be made
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_reset(db_lh2_t *lh2);

#endif /* LH2_H_ */
