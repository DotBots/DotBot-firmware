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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <nrf.h>
#include "ipc.h"

__NO_RETURN extern void reset_handler(void);
__NO_RETURN void dummy_handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void exit(int status);

// Exceptions handlers
__attribute__ ((weak, alias("dummy_handler"))) void NMI_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void MemManage_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void BusFault_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void UsageFault_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SVC_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void DebugMon_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PendSV_Handler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SysTick_Handler(void);

void HardFault_Handler(void);

// External interrupts handlers
__attribute__ ((weak, alias("dummy_handler"))) void FPU_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void CACHE_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPU_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void CLOCK_POWER_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SERIAL0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SERIAL1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPIM4_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SERIAL2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SERIAL3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void GPIOTE0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SAADC_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RTC0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RTC1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void WDT0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void WDT1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU4_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void EGU5_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PDM0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void I2S0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void IPC_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void QSPI_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void NFCT_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void GPIOTE1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void QDEC0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void QDEC1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void USBD_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void USBREGULATOR_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void KMU_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void CRYPTOCELL_IRQHandler(void);

#ifndef NO_IPC
volatile __attribute__((section(".shared_data"))) ipc_shared_data_t ipc_shared_data;

void db_ipc_network_call(ipc_req_t req) {
    if (req != DB_IPC_REQ_NONE) {
        ipc_shared_data.req                    = req;
        NRF_IPC_S->TASKS_SEND[DB_IPC_CHAN_REQ] = 1;
    }
    while (!ipc_shared_data.net_ack) {
        if (ipc_shared_data.req == DB_IPC_REQ_NONE) {
            // Something went wrong and, the net-core deleted the request without fulfilling it.
            // Re-send it
            ipc_shared_data.req                    = req;
            NRF_IPC_S->TASKS_SEND[DB_IPC_CHAN_REQ] = 1;
        }
    }
    ipc_shared_data.net_ack = false;
};

void release_network_core(void) {
    // Do nothing if network core is already started and ready
    if (!NRF_RESET_S->NETWORK.FORCEOFF && ipc_shared_data.net_ready) {
        return;
    } else if (!NRF_RESET_S->NETWORK.FORCEOFF) {
        ipc_shared_data.net_ready = false;
    }

    NRF_RESET_S->NETWORK.FORCEOFF = (RESET_NETWORK_FORCEOFF_FORCEOFF_Release << RESET_NETWORK_FORCEOFF_FORCEOFF_Pos);

    while (!ipc_shared_data.net_ready) {}
}
#endif

// Vector table
extern uint32_t __stack_end__;
typedef void(*vector_table_t)(void);
#if defined(IRQ_VECTORS_SHORT)
#define VECTORS_SIZE    30
#else
#define VECTORS_SIZE    85
#endif
extern const vector_table_t _vectors[VECTORS_SIZE];
const vector_table_t _vectors[] __attribute__((used, section(".vectors"))) = {
    (vector_table_t)&__stack_end__,     //     Initial Stack Pointer
    reset_handler,                      //     Reset Handler
    NMI_Handler,                        // -14 NMI Handler
    HardFault_Handler,                  // -13 Hard Fault HandleR
    MemManage_Handler,                  // -12 MPU Fault Handler
    BusFault_Handler,                   // -11 Bus Fault Handler
    UsageFault_Handler,                 // -10 Usage Fault Handler
    0,                                  //     Reserved
    0,                                  //     Reserved
    0,                                  //     Reserved
    0,                                  //     Reserved
    SVC_Handler,                        //  -5 SVCall Handler
    DebugMon_Handler,                   //  -4 Debug Monitor Handler
    0,                                  //     Reserved
    PendSV_Handler,                     //  -2 PendSV Handler
    SysTick_Handler,                    //  -1 SysTick Handler

    // External Interrupts
    FPU_IRQHandler,
    CACHE_IRQHandler,
    0,
    SPU_IRQHandler,
    0,
    CLOCK_POWER_IRQHandler,
    0,
    0,
    SERIAL0_IRQHandler,
    SERIAL1_IRQHandler,
    SPIM4_IRQHandler,
    SERIAL2_IRQHandler,
    SERIAL3_IRQHandler,
    GPIOTE0_IRQHandler,
#if !defined(IRQ_VECTORS_SHORT)
    SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    0,
    0,
    RTC0_IRQHandler,
    RTC1_IRQHandler,
    0,
    0,
    WDT0_IRQHandler,
    WDT1_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    EGU0_IRQHandler,
    EGU1_IRQHandler,
    EGU2_IRQHandler,
    EGU3_IRQHandler,
    EGU4_IRQHandler,
    EGU5_IRQHandler,
    PWM0_IRQHandler,
    PWM1_IRQHandler,
    PWM2_IRQHandler,
    PWM3_IRQHandler,
    0,
    PDM0_IRQHandler,
    0,
    I2S0_IRQHandler,
    0,
    IPC_IRQHandler,
    QSPI_IRQHandler,
    0,
    NFCT_IRQHandler,
    0,
    GPIOTE1_IRQHandler,
    0,
    0,
    0,
    QDEC0_IRQHandler,
    QDEC1_IRQHandler,
    0,
    USBD_IRQHandler,
    USBREGULATOR_IRQHandler,
    0,
    KMU_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    CRYPTOCELL_IRQHandler,
#endif
};

// Exception handlers
void HardFault_Handler(void) {
   while(1) {
       __NOP();
   }
}

void dummy_handler(void) {
   while(1) {
       __NOP();
   }
}
