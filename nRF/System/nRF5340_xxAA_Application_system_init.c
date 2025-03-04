/*

Copyright (c) 2009-2021 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.

*/

/* NOTE: Template files (including this one) are application specific and therefore expected to
   be copied into the application project folder prior to its use! */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf53_erratas.h"
#include "system_nrf5340_application.h"
#include "system_nrf53_approtect.h"
#include "tz.h"

/*lint ++flb "Enter library region" */

void SystemStoreFICRNS(void);
void SystemLockFICRNS(void);

void system_init(void)
{
    /* Perform Secure-mode initialization routines. */

    /* Set all ARM SAU regions to NonSecure if TrustZone extensions are enabled.
    * Nordic SPU should handle Secure Attribution tasks */
    #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
        SAU->CTRL |= (1 << SAU_CTRL_ALLNS_Pos);
    #endif

    /* Workaround for Errata 97 "ERASEPROTECT, APPROTECT, or startup problems" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_97())
    {
        if (*((volatile uint32_t *)0x50004A20ul) == 0)
        {
            *((volatile uint32_t *)0x50004A20ul) = 0xDul;
            *((volatile uint32_t *)0x5000491Cul) = 0x1ul;
            *((volatile uint32_t *)0x5000491Cul) = 0x0ul;
        }
    }

    /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim
     until one ADDR is not initialized. */
    uint32_t index = 0;
    for (index = 0; index < 32ul && NRF_FICR_S->TRIMCNF[index].ADDR != (uint32_t *)0xFFFFFFFFul; index++){
        #if defined ( __ICCARM__ )
            /* IAR will complain about the order of volatile pointer accesses. */
            #pragma diag_suppress=Pa082
        #endif
        *((volatile uint32_t *)NRF_FICR_S->TRIMCNF[index].ADDR) = NRF_FICR_S->TRIMCNF[index].DATA;
        #if defined ( __ICCARM__ )
            #pragma diag_default=Pa082
        #endif
    }

    /* errata 64 must be before errata 42, as errata 42 is dependant on the changes in errata 64*/
    /* Workaround for Errata 64 "VREGMAIN has invalid configuration when CPU is running at 128 MHz" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_64())
    {
        *((volatile uint32_t *)0x5000470Cul) = 0x29ul;
        *((volatile uint32_t *)0x5000473Cul) = 0x3ul;
    }

    /* Workaround for Errata 42 "Reset value of HFCLKCTRL is invalid" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_42())
    {
        *((volatile uint32_t *)0x50039530ul) = 0xBEEF0044ul;
        NRF_CLOCK_S->HFCLKCTRL = CLOCK_HFCLKCTRL_HCLK_Div2 << CLOCK_HFCLKCTRL_HCLK_Pos;
    }

    /* Workaround for Errata 46 "Higher power consumption of LFRC" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_46())
    {
        *((volatile uint32_t *)0x5003254Cul) = 0;
    }

    /* Workaround for Errata 49 "SLEEPENTER and SLEEPEXIT events asserted after pin reset" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_49())
    {
        if (NRF_RESET_S->RESETREAS & RESET_RESETREAS_RESETPIN_Msk)
        {
            NRF_POWER_S->EVENTS_SLEEPENTER = 0;
            NRF_POWER_S->EVENTS_SLEEPEXIT = 0;
        }
    }

    /* Workaround for Errata 55 "Bits in RESETREAS are set when they should not be" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_55())
    {
        if (NRF_RESET_S->RESETREAS & RESET_RESETREAS_RESETPIN_Msk){
            NRF_RESET_S->RESETREAS = ~RESET_RESETREAS_RESETPIN_Msk;
        }
    }

    /* Workaround for Errata 69 "VREGMAIN configuration is not retained in System OFF" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
    if (nrf53_errata_69())
    {
        *((volatile uint32_t *)0x5000470Cul) =0x65ul;
    }

    #if !defined(NRF_SKIP_FICR_NS_COPY_TO_RAM)
        SystemStoreFICRNS();
    #endif

    #if defined(CONFIG_NFCT_PINS_AS_GPIOS)
        if ((NRF_UICR_S->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
        {
            NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
            NRF_UICR_S->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
            while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
            NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
            NVIC_SystemReset();
        }
    #endif

    /* Enable SWO trace functionality. If ENABLE_SWO is not defined, SWO pin will be used as GPIO (see Product
       Specification to see which one). */
    #if defined (ENABLE_SWO)
        // Enable Trace And Debug peripheral
        NRF_TAD_S->ENABLE = TAD_ENABLE_ENABLE_Msk;
        NRF_TAD_S->CLOCKSTART = TAD_CLOCKSTART_START_Msk;
        // Set up Trace pad SPU firewall
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACEDATA0_PIN);
        // Configure trace port pad
        NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;
        // Select trace pin
        NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;
        // Set trace port speed to 64 MHz
        NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_64MHz;
    #endif

    /* Enable Trace functionality. If ENABLE_TRACE is not defined, TRACE pins will be used as GPIOs (see Product
       Specification to see which ones). */
    #if defined (ENABLE_TRACE)
        // Enable Trace And Debug peripheral
        NRF_TAD_S->ENABLE = TAD_ENABLE_ENABLE_Msk;
        NRF_TAD_S->CLOCKSTART = TAD_CLOCKSTART_START_Msk;
        // Set up Trace pads SPU firewall
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACECLK_PIN);
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACEDATA0_PIN);
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACEDATA1_PIN);
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACEDATA2_PIN);
        NRF_SPU_S->GPIOPORT[0].PERM &= ~(1 << TRACE_TRACEDATA3_PIN);
        // Configure trace port pads
        NRF_P0_S->PIN_CNF[TRACE_TRACECLK_PIN] =   TRACE_PIN_CNF_VALUE;
        NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;
        NRF_P0_S->PIN_CNF[TRACE_TRACEDATA1_PIN] = TRACE_PIN_CNF_VALUE;
        NRF_P0_S->PIN_CNF[TRACE_TRACEDATA2_PIN] = TRACE_PIN_CNF_VALUE;
        NRF_P0_S->PIN_CNF[TRACE_TRACEDATA3_PIN] = TRACE_PIN_CNF_VALUE;
        // Select trace pins
        NRF_TAD_S->PSEL.TRACECLK   = TRACE_TRACECLK_PIN;
        NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;
        NRF_TAD_S->PSEL.TRACEDATA1 = TRACE_TRACEDATA1_PIN;
        NRF_TAD_S->PSEL.TRACEDATA2 = TRACE_TRACEDATA2_PIN;
        NRF_TAD_S->PSEL.TRACEDATA3 = TRACE_TRACEDATA3_PIN;
        // Set trace port speed to 64 MHz
        NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_64MHz;
    #endif

    /* Allow Non-Secure code to run FPU instructions.
     * If only the secure code should control FPU power state these registers should be configured accordingly in the secure application code. */
    SCB->NSACR |= (3UL << 10);

    // This function is only called when running in secure mode, so set back to secure all the peripherals that might be not secure at startup (because of swarmit)
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_I2S0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_I2S0].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
                                            SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_P0_P1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PDM0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PDM0].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
                                            SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_COMP_LPCOMP].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU2].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU3].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU4].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_EGU5].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM0].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM1].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM2].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM2].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM3].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_PWM3].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_QDEC0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_QDEC1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_QSPI].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_QSPI].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_RTC0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_RTC1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SAADC].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SAADC].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM0_SPIS0_TWIM0_TWIS0_UARTE0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM0_SPIS0_TWIM0_TWIS0_UARTE0].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM1_SPIS1_TWIM1_TWIS1_UARTE1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM1_SPIS1_TWIM1_TWIS1_UARTE1].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM2_SPIS2_TWIM2_TWIS2_UARTE2].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM2_SPIS2_TWIM2_TWIS2_UARTE2].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM3_SPIS3_TWIM3_TWIS3_UARTE3].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM3_SPIS3_TWIM3_TWIS3_UARTE3].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM4].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM4].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_TIMER0].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_TIMER1].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_TIMER2].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_USBD].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_USBD].PERM |= (SPU_PERIPHID_PERM_DMA_NoSeparateAttribute << SPU_PERIPHID_PERM_DMA_Pos |
        SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_USBREGULATOR].PERM = SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos;

    /* Handle fw-branch APPROTECT setup. */
    nrf53_handle_approtect();

    /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
    * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
    * operations are not used in your code. */
    #if (__FPU_USED == 1)
        SCB->CPACR |= (3UL << 20) | (3UL << 22);
        __DSB();
        __ISB();
    #endif
}

/* Workaround to allow NS code to access FICR. Override NRF_FICR_NS to move FICR_NS buffer. */
#define FICR_SIZE 0x1000ul
#define RAM_BASE 0x20000000ul
#define RAM_END  0x2FFFFFFFul

/* Copy FICR_S to FICR_NS RAM region */
void SystemStoreFICRNS(void)
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }
    /* Copy FICR to NS-accessible RAM block. */
    volatile uint32_t * from            = (volatile uint32_t *)((uint32_t)NRF_FICR_S + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * to              = (volatile uint32_t *)((uint32_t)NRF_FICR_NS + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * copy_from_end   = (volatile uint32_t *)NRF_FICR_S;
    while (from >= copy_from_end)
    {
        *(to--) = *(from--);
    }

    /* Make RAM region NS. */
    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    NRF_SPU_S->RAMREGION[ram_region].PERM &= ~(1 << SPU_RAMREGION_PERM_SECATTR_Pos);
}

/* Block write and execute access to FICR RAM region */
void SystemLockFICRNS(void)
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }

    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    NRF_SPU_S->RAMREGION[ram_region].PERM &=
        ~(
            (1 << SPU_RAMREGION_PERM_WRITE_Pos) |
            (1 << SPU_RAMREGION_PERM_EXECUTE_Pos)
        );
    NRF_SPU_S->RAMREGION[ram_region].PERM |= 1 << SPU_RAMREGION_PERM_LOCK_Pos;
}

/*lint --flb "Leave library region" */
