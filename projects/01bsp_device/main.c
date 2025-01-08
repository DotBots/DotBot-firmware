/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Simple application used to read device information (ID).
 *
 * @copyright Inria, 2024
 *
 */
#include <nrf.h>
#include <stdio.h>
#include "device.h"

int main(void) {
    puts("Device info");
    puts("===========");
    printf("CPU\t: ");
#if defined(NRF5340_XXAA_APPLICATION)
    puts("nRF5340-Application");
#elif defined(NRF5340_XXAA_NETWORK)
    puts("nRF5340-Network");
#elif defined(NRF52840_XXAA)
    puts("nRF52840_XXAA");
#elif defined(NRF52833_XXAA)
    puts("nRF52833_XXAA");
#else
    puts("Unknown");
#endif
#if !(defined(NRF5340_XXAA) && defined(NRF_APPLICATION))
    printf("Addr\t: %016llX\n", db_device_addr());
#endif
    printf("ID\t: %016llX\n", db_device_id());

    while (1) {
        __WFE();
    }
}
