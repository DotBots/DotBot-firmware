/**
 * @file
 * @ingroup bsp_rng
 *
 * @brief  nRF52833-specific definition of the "rng" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "rng_nrf5340_app.c"
#else
#include "rng_default.c"
#endif
