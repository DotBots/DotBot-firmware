/**
 * @file
 * @defgroup project_nrf5340_app_core    nRF5340 application core
 * @ingroup projects
 * @brief This application is used to configure some peripherals as non secure and power on the network core
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 */

#include <nrf.h>

#include "clock.h"
#include "ipc.h"
#include "tz.h"

int main(void) {

    // On nrf53 configure constant latency mode for better performances
    NRF_POWER_S->TASKS_CONSTLAT = 1;

    db_hfclk_init();
    db_lfclk_init();

    // Mark peripherals required by the network as non secure

    // VREQCTRL (address at 0x41004000 => periph ID is 4)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_VREQCTRL);

    // POWER (address at 0x41005000 => periph ID is 5)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_CLOCK_POWER_RESET);

    // RADIO (address at 0x41008000 => periph ID is 8)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_RADIO);
    db_tz_enable_network_periph_dma(NRF_NETWORK_PERIPH_ID_RADIO);

    // RNG (address at 0x41009000 => periph ID is 9)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_RNG);

    // TIMER0 (address at 0x4100C000 => periph ID is 12)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_TIMER0);

    // UARTE0/TWIM0/SPIM0 (address at 0x41013000 => periph ID is 19)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_SPIM0_SPIS0_TWIM0_TWIS0_UARTE0);
    db_tz_enable_network_periph_dma(NRF_NETWORK_PERIPH_ID_SPIM0_SPIS0_TWIM0_TWIS0_UARTE0);

    // RTC1 (address at 0x41016000 => periph ID is 22)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_RTC1);

    // TIMER1 (address at 0x41018000 => periph ID is 24)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_TIMER1);

    // TIMER2 (address at 0x41019000 => periph ID is 25)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_TIMER2);

    // WDT (address at 0x4100B000 => periph ID is 11)
    db_tz_enable_network_periph(NRF_NETWORK_PERIPH_ID_WDT0);

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
