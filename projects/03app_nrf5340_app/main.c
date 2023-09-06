/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to configure some peripherals as non secure and power on the network core
 *
 * @copyright Inria, 2023
 *
 */
#include <nrf.h>

#include "clock.h"
#include "ipc.h"

int main(void) {

    // On nrf53 configure constant latency mode for better performances
    NRF_POWER_S->TASKS_CONSTLAT = 1;

    db_hfclk_init();
    db_lfclk_init();

    // Mark peripherals required by the network as non secure

    // VREQCTRL (address at 0x41004000 => periph ID is 4)
    NRF_SPU_S->PERIPHID[4].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // POWER (address at 0x41005000 => periph ID is 5)
    NRF_SPU_S->PERIPHID[5].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // RADIO (address at 0x41008000 => periph ID is 8)
    NRF_SPU_S->PERIPHID[8].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos |
                                   SPU_PERIPHID_PERM_DMASEC_NonSecure << SPU_PERIPHID_PERM_DMASEC_Pos);

    // RNG (address at 0x41009000 => periph ID is 9)
    NRF_SPU_S->PERIPHID[9].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                   SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                   SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // TIMER0 (address at 0x4100C000 => periph ID is 12)
    NRF_SPU_S->PERIPHID[12].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // UARTE0/TWIM0/SPIM0 (address at 0x41013000 => periph ID is 19)
    NRF_SPU_S->PERIPHID[19].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos |
                                    SPU_PERIPHID_PERM_DMASEC_NonSecure << SPU_PERIPHID_PERM_DMASEC_Pos);

    // RTC1 (address at 0x41016000 => periph ID is 22)
    NRF_SPU_S->PERIPHID[22].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // TIMER1 (address at 0x41018000 => periph ID is 24)
    NRF_SPU_S->PERIPHID[24].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // TIMER2 (address at 0x41019000 => periph ID is 25)
    NRF_SPU_S->PERIPHID[25].PERM = (SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                                    SPU_PERIPHID_PERM_SECATTR_NonSecure << SPU_PERIPHID_PERM_SECATTR_Pos |
                                    SPU_PERIPHID_PERM_PRESENT_IsPresent << SPU_PERIPHID_PERM_PRESENT_Pos);

    // Define FLASHREGION 62-64 as non secure (half of the network core flash)
    for (uint8_t region = 62; region < 64; region++) {
        NRF_SPU_S->FLASHREGION[region].PERM = (SPU_FLASHREGION_PERM_READ_Enable << SPU_FLASHREGION_PERM_READ_Pos |
                                               SPU_FLASHREGION_PERM_WRITE_Enable << SPU_FLASHREGION_PERM_WRITE_Pos |
                                               SPU_FLASHREGION_PERM_SECATTR_Non_Secure << SPU_FLASHREGION_PERM_SECATTR_Pos);
    }

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
