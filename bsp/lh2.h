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
    DB_LH2_RUNNING,         ///< the lh2 engine is running
    DB_LH2_RAW_DATA_READY,  ///< some lh2 raw data is available
    DB_LH2_LOCATION_READY,  ///< some lh2 location is ready to be read
} db_lh2_state_t;

typedef struct {
    uint64_t bits_sweep;           ///< bits sweep is the result of the demodulation, sweep_N indicates which SPI transfer those bits are associated with
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    int8_t   bit_offset;           ///< bit_offset indicates an offset between the start of the packet, as indicated by envelope dropping, and the 17-bit sequence that is verified to be in a known LFSR sequence
} db_lh2_raw_data_t;

typedef struct {
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    uint32_t lfsr_location;        ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
} db_lh2_location_t;

typedef struct {
    db_lh2_state_t    state;                           ///< current state of the lh2 engine
    db_lh2_raw_data_t raw_data[LH2_LOCATIONS_COUNT];   ///< raw data decoded from the lighthouse
    db_lh2_location_t locations[LH2_LOCATIONS_COUNT];  ///< buffer holding the computed locations
} db_lh2_t;

//=========================== public ===========================================

/**
 * @brief Initialize LH2
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void db_lh2_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief Process raw data coming from the lighthouse
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_process_raw_data(db_lh2_t *lh2);

/**
 * @brief Compute the location based on available raw data
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
