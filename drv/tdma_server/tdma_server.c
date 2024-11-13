/**
 * @file
 * @ingroup     drv_tdma_server
 *
 * @brief       Driver for Time-Division-Multiple-Access for the Gateway radio
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2024
 */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "tdma_server_nrf5340_app.c"
#else
#include "tdma_server_default.c"
#endif
