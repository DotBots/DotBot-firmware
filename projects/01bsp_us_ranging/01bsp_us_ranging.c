/**
 * @file 01bsp_us_ranging.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is a short example of how to do Ultrasound ranging with HC-SR04 sensor on the DotBot board.
 * This code currently works on nRF52840. To use the code on DotBot you need to define different GPIO for US sensor ON and READ pin (trigger and echo) in hc_sr04.c
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "hc_sr04.h"


//=========================== defines =========================================

//=========================== prototypes =========================================

void us_callback(uint32_t us_reading);     // callback for the US echo measurement
void timer_callback(void);                 // callback for timer0 after pulse_offset_ms + pulse_duration_ms

//=========================== variables =========================================


NRF_TIMER_Type *timer0;                   // variable for passing the Timer for sensor trigger 
NRF_TIMER_Type *timer1;                   // variable for passing the Timer for sensor echo measurement
 

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    timer0 = NRF_TIMER0;
    timer1 = NRF_TIMER1;

    // uncomment if using this code on DotBot Turn ON the DotBot board regulator
    //db_board_init();

    // initilize the US sensor, set callback and chose the timers
    hc_sr04_init(&us_callback, &timer_callback, timer0, timer1);
   
    // start ranging
    hc_sr04_start();
    
    // start high frequency clock needed for PPI
    hfclk_init();

    while (1) {
        
        __WFE();      
        __SEV();
        __WFE();

    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

/**
 *  @brief This is a callback for US reading.
 *         The code ends up here every time a new measurement is taken. The new measurement is stored in us_reading argument
 */
void us_callback(uint32_t us_reading) {

    __NOP();

}

/**
 *  @brief This is a callback for the US trigger timer.
 *         The code ends up here every pulse_offset_ms + pulse_duration_ms times.
 *         We could change the pulse_offset_ms here by writting a new value to timer0->CC[0] and timer0->CC[1]
 */
void timer_callback(void) {

    __NOP();

}
    


