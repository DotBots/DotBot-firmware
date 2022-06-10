/**
 * @file 01bsp_us_ranging.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 * @brief This is a short example of how to do Ultrasound ranging with HC-SR04 sensor on the DotBot board.
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

void us_callback(uint32_t us_reading);
void timer_callback(void);

//=========================== variables =========================================

NRF_TIMER_Type *timer0;
NRF_TIMER_Type *timer1;

double          pulse_duration_ms;
volatile double pulse_offset_ms;

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    pulse_duration_ms  = 0.01;
    pulse_offset_ms    = 250;

    timer0 = NRF_TIMER0;
    timer1 = NRF_TIMER1;

    // initilize the US sensor, set callback and chose the timers
    us_init(&us_callback, &timer_callback, timer0, timer1);
   
    // set compare values for the US on timer
    us_on_set_trigger(pulse_duration_ms, pulse_offset_ms);

    // start ranging
    us_start();


    while (1) {
        
        __WFE();      
        __SEV();
        __WFE();

    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}


void us_callback(uint32_t us_reading) {

    __NOP();

}

void timer_callback(void) {

    __NOP();

}
    


