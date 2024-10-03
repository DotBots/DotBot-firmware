/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF52833-specific definition of the "lh2" bsp module.
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "lh2_nrf5340_net.c"
#else
#include "lh2_default.c"
#endif
