/**
 * @file
 * @ingroup     drv_tdma_client
 *
 * @brief       Driver for Time-Division-Multiple-Access fot the DotBot radio
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2024
 */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "tdma_client_nrf5340_app.c"
#else
#include "tdma_client_default.c"
#endif
