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
 __attribute__ ((weak, alias("dummy_handler"))) void CLOCK_POWER_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void RADIO_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void RNG_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void GPIOTE_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void WDT_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void TIMER0_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void ECB_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void AAR_CCM_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void TEMP_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void RTC0_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void IPC_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void SERIAL0_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void EGU0_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void RTC1_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void TIMER1_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void TIMER2_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void SWI0_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void SWI1_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void SWI2_IRQHandler(void);
 __attribute__ ((weak, alias("dummy_handler"))) void SWI3_IRQHandler(void);

// Vector table
extern uint32_t __stack_end__;
typedef void(*vector_table_t)(void);
extern const vector_table_t _vectors[48];
const vector_table_t _vectors[48] __attribute__((used, section(".vectors"))) = {
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
    0,
    0,
    0,
    0,
    0,
    CLOCK_POWER_IRQHandler,
    0,
    0,
    RADIO_IRQHandler,
    RNG_IRQHandler,
    GPIOTE_IRQHandler,
    WDT_IRQHandler,
    TIMER0_IRQHandler,
    ECB_IRQHandler,
    AAR_CCM_IRQHandler,
    0,
    TEMP_IRQHandler,
    RTC0_IRQHandler,
    IPC_IRQHandler,
    SERIAL0_IRQHandler,
    EGU0_IRQHandler,
    0,
    RTC1_IRQHandler,
    0,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    SWI0_IRQHandler,
    SWI1_IRQHandler,
    SWI2_IRQHandler,
    SWI3_IRQHandler,
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
