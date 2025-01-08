/**
 * @file
 * @ingroup bsp_cpu
 *
 * @brief  Generic implementation of the cpu support.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2024-present
 */

/**
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Startup code and vectors definition
 *
 * @copyright Inria, 2024
 *
 */

#include <stdint.h>
#include <nrf.h>

extern __NO_RETURN int main(void);
extern void system_init(void);

extern uint32_t __data_load_start__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __text_load_start__;
extern uint32_t __text_start__;
extern uint32_t __text_end__;
extern uint32_t __fast_load_start__;
extern uint32_t __fast_start__;
extern uint32_t __fast_end__;
extern uint32_t __ctors_load_start__;
extern uint32_t __ctors_start__;
extern uint32_t __ctors_end__;
extern uint32_t __dtors_load_start__;
extern uint32_t __dtors_start__;
extern uint32_t __dtors_end__;
extern uint32_t __rodata_load_start__;
extern uint32_t __rodata_start__;
extern uint32_t __rodata_end__;
extern uint32_t __tdata_load_start__;
extern uint32_t __tdata_start__;
extern uint32_t __tdata_end__;

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __tbss_start__;
extern uint32_t __tbss_end__;

extern uint32_t __heap_start__;
extern uint32_t __heap_end__;

extern uint32_t __stack_start__;
extern uint32_t __stack_end__;
extern uint32_t __stack_process_start__;
extern uint32_t __stack_process_end__;
extern uint32_t __HEAPSIZE__;
extern uint32_t __STACKSIZE__;
extern uint32_t __STACKSIZE_PROCESS__;

void HardFault_Handler(void);

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "nRF5340_xxAA_Application_cpu.c"
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#include "nRF5340_xxAA_Network_cpu.c"
#elif defined(NRF52840_XXAA)
#include "nRF52840_xxAA_cpu.c"
#elif defined(NRF52833_XXAA)
#include "nRF52833_xxAA_cpu.c"
#else
#error "Unsupported CPU"
#endif

// Fix compile issue with Thread-Local Storage (https://wiki.segger.com/Thread-Local_Storage)
void __aeabi_read_tp(void) {}

static void _copy(uint32_t *dst, uint32_t *src, uint32_t *end) {
    while(dst < end) {
        *dst++ = *src++;
    }
}

static void _zero(uint32_t *dst, uint32_t *end) {
    while(dst < end) {
        *dst++ = 0;
    }
}

// Entry point
void reset_handler(void) {
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  /* set CP10 and CP11 Full Access */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
    __set_MSP((uint32_t)&__stack_end__);
    __set_PSP((uint32_t)&__stack_process_end__);
    __set_CONTROL(0);
#endif

    _copy(&__data_start__, &__data_load_start__, &__data_end__);

    uint32_t *src;
    uint32_t *dst;

#if defined(DEBUG)
    src = &__text_load_start__;
    dst = &__text_start__;
    while(dst < &__text_end__) {
        if (dst == src) {
            break;
        }
        *dst++ = *src++;
    }
#endif
    _copy(&__fast_start__, &__fast_load_start__, &__fast_end__);
    _copy(&__ctors_start__, &__ctors_load_start__, &__ctors_end__);
    _copy(&__dtors_start__, &__dtors_load_start__, &__dtors_end__);

#if defined(DEBUG)
    src = &__rodata_load_start__;
    dst = &__rodata_start__;
    while(dst < &__rodata_end__) {
        if (dst == src) {
            break;
        }
        *dst++ = *src++;
    }
#endif
    src = &__tdata_load_start__;
    dst = &__tdata_start__;
    while(dst < &__tdata_end__) {
        if (dst == src) {
            *dst++ = *src++;
        }
    }

    // Zeroing bss data
    _zero(&__bss_start__, &__bss_end__);
    _zero(&__tbss_start__, &__tbss_end__);

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION) && !defined(USE_SWARMIT)
    extern uint32_t __shared_data_start__;
    extern uint32_t __shared_data_end__;
    _zero(&__shared_data_start__, &__shared_data_end__);
#endif

    // Calling constructors
    typedef void (*ctor_func_t)(void);
    ctor_func_t func = (ctor_func_t)&__ctors_start__;
    while(&func < (ctor_func_t *)&__ctors_end__) {
        func++();
    }

#if !defined(__NO_SYSTEM_INIT) && !defined(NRF_TRUSTZONE_NONSECURE)
    system_init();
#endif

    main();
}
