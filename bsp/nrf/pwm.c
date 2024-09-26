/**
 * @file
 * @ingroup bsp_pwm
 *
 * @brief  nRF52833-specific definition of the "pwm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#if defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "pwm_nrf5340_net.c"
#else
#include "pwm_default.c"
#endif
