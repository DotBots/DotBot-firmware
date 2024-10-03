/**
 * @file
 * @ingroup bsp_radio
 *
 * @brief  nRF52833-specific definition of the "radio" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "radio_nrf5340_app.c"
#else
#include "radio_default.c"
#endif
