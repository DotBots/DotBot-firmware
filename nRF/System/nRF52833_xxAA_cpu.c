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
__attribute__ ((weak, alias("dummy_handler"))) void POWER_CLOCK_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RADIO_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void UARTE0_UART0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void NFCT_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void GPIOTE_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SAADC_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RTC0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TEMP_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RNG_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void ECB_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void CCM_AAR_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void WDT_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RTC1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void QDEC_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI0_EGU0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI1_EGU1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI2_EGU2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI3_EGU3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI4_EGU4_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SWI5_EGU5_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void TIMER4_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM0_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PDM_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void MWU_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPIM2_SPIS2_SPI2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void RTC2_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void I2S_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void FPU_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void USBD_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void UARTE1_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void PWM3_IRQHandler(void);
__attribute__ ((weak, alias("dummy_handler"))) void SPIM3_IRQHandler(void);

// Vector table
extern uint32_t __stack_end__;
typedef void(*vector_table_t)(void);
extern const vector_table_t _vectors[64];
const vector_table_t _vectors[64] __attribute__((used, section(".vectors"))) = {
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
    POWER_CLOCK_IRQHandler,
    RADIO_IRQHandler,
    UARTE0_UART0_IRQHandler,
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,
    NFCT_IRQHandler,
    GPIOTE_IRQHandler,
    SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    CCM_AAR_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    SWI0_EGU0_IRQHandler,
    SWI1_EGU1_IRQHandler,
    SWI2_EGU2_IRQHandler,
    SWI3_EGU3_IRQHandler,
    SWI4_EGU4_IRQHandler,
    SWI5_EGU5_IRQHandler,
    TIMER3_IRQHandler,
    TIMER4_IRQHandler,
    PWM0_IRQHandler,
    PDM_IRQHandler,
    0,
    0,
    MWU_IRQHandler,
    PWM1_IRQHandler,
    PWM2_IRQHandler,
    SPIM2_SPIS2_SPI2_IRQHandler,
    RTC2_IRQHandler,
    I2S_IRQHandler,
    FPU_IRQHandler,
    USBD_IRQHandler,
    UARTE1_IRQHandler,
    0,
    0,
    0,
    0,
    PWM3_IRQHandler,
    0,
    SPIM3_IRQHandler,
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
