/**
 * @file
 * @defgroup project_nrf5340_app_core    nRF5340 application core
 * @ingroup projects
 * @brief This application is used to configure some peripherals as non secure and power on the network core
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include <nrf.h>

#include "tz.h"

int main(void) {

    // Define FLASHREGION 62-63 as non secure (half of the network core flash)
    db_configure_flash_non_secure(62, 2);

    // Configure non secure DPPI channels
    NRF_SPU_S->DPPI[0].PERM = 0;

    // Mark both GPIO ports as non secure so all of their pins can be controlled by the network core
    NRF_SPU_S->GPIOPORT[0].PERM = 0;
    NRF_SPU_S->GPIOPORT[1].PERM = 0;

    // Give access to all GPIOs to network core
    for (uint8_t pin = 0; pin < 32; pin++) {
        if (pin > 1) {  // P0.0 and P0.1 are used for the LFXO crystal
            NRF_P0_S->PIN_CNF[pin] = GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
        }
        NRF_P1_S->PIN_CNF[pin] = GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
    }

    // Release the network core
    NRF_RESET_S->NETWORK.FORCEOFF = 0;

    while (1) {
        __WFE();
    };
}
