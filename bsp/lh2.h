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

//=========================== defines =========================================

#define LH2_LOCATIONS_COUNT                    4
#define LH2_RESULTS_SIZE                       LH2_LOCATIONS_COUNT * 2

typedef enum {
    DB_LH2_IDLE,
    DB_LH2_RUNNING,
    DB_LH2_READY,
} db_lh2_state_t;

typedef struct {
    db_lh2_state_t state;
    uint32_t results[LH2_RESULTS_SIZE];
} db_lh2_t;

//=========================== variables =========================================

//=========================== public ==========================================

// initialization function
void db_lh2_init(void);

// do fil's stuff
void db_lh2_process_location(db_lh2_t *lh2);

void db_lh2_reset(db_lh2_t *lh2);

// start transfering bits from the lighthouse module
void db_lh2_start_transfer(db_lh2_t *lh2);
// stop transfering bits from the lighthouse module
void db_lh2_stop_transfer(db_lh2_t *lh2);

#endif /* LH2_H_ */
