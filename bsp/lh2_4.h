#ifndef __LH2_H_
#define __LH2_H_

/**
 * @defgroup    bsp_lh2 LightHouse 2 support
 * @ingroup     bsp
 * @brief       Control the LH2 sensor
 *
 * @{
 * @file
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 20232
 * @}
 */

#include <nrf.h>
#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"

//=========================== defines ==========================================

#define LH2_4_LOCATIONS_COUNT 2  ///< Number of computed locations
#define LH2_4_BASESTATION_COUNT 4  ///< Number of supported concurrent basestations

/// LH2 internal state
typedef enum {
    DB_LH2_4_IDLE,            ///< the lh2 engine is idle
    DB_LH2_4_RUNNING,         ///< the lh2 engine is running
    DB_LH2_4_RAW_DATA_READY,  ///< some lh2 raw data is available
    DB_LH2_4_LOCATION_READY,  ///< some lh2 location is ready to be read
} db_lh2_4_state_t;

/// LH2 data ready buffer state
typedef enum {
    DB_LH2_4_NO_NEW_DATA,            ///< The data occupying this spot of the buffer has already been sent.
    DB_LH2_4_DATA_READY,             ///< The data occupying this spot of the buffer is new and ready to send.
} db_lh2_4_data_ready_state_t;

/// LH2 raw data
typedef struct __attribute__((packed)) {
    uint64_t bits_sweep;           ///< bits sweep is the result of the demodulation, sweep_N indicates which SPI transfer those bits are associated with
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    int8_t   bit_offset;           ///< bit_offset indicates an offset between the start of the packet, as indicated by envelope dropping, and the 17-bit sequence that is verified to be in a known LFSR sequence
} db_lh2_4_raw_data_t;

/// LH2 raw data location
typedef struct __attribute__((packed)) {
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    uint32_t lfsr_location;        ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
} db_lh2_4_location_t;

/// LH2 instance
typedef struct {
    db_lh2_4_state_t    state;                                          ///< current state of the lh2 engine
    db_lh2_4_raw_data_t raw_data[2][LH2_4_BASESTATION_COUNT];           ///< raw data decoded from the lighthouse
    db_lh2_4_location_t locations[2][LH2_4_BASESTATION_COUNT];          ///< buffer holding the computed locations
    uint32_t            timestamps[2][LH2_4_BASESTATION_COUNT];         ///< timestamp of when the raw data was received
    db_lh2_4_data_ready_state_t data_ready[2][LH2_4_BASESTATION_COUNT]; ///< Is the data in the buffer ready to send over radio, or has it already been sent ?
} db_lh2_4_t;

//=========================== public ===========================================

/**
 * @brief Initialize LH2
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void db_lh2_4_init(db_lh2_4_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief Process raw data coming from the lighthouse
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_4_process_raw_data(db_lh2_4_t *lh2);

/**
 * @brief Compute the location based on available raw data
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_4_process_location(db_lh2_4_t *lh2);

/**
 * @brief Start the LH2 frame acquisition
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_4_start(db_lh2_4_t *lh2);

/**
 * @brief Stop the LH2 frame acquisition
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_4_stop(db_lh2_4_t *lh2);

/**
 * @brief Reset the lh2 internal state so new location computation can be made
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_4_reset(db_lh2_4_t *lh2);

#endif /* __LH2_H_ */
