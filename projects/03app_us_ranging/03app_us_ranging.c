/**
 * @file 03app_us_ranging.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is an example application for Ultrasound ranging with HC-SR04 sensor on the DotBot.
 * This app receives a radio packet from the gateway, parse the packet and sets up the Timer offset for triggering the US ranging.
 * The US readings are then transmitted back to the gateway by radio.
 * This code currently works on nRF52840. 
 *
 * To use the code on DotBot you need to define different GPIO for US sensor ON and READ pin (trigger and echo) in hc_sr04.c
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

// uncoment the board.h when porting this code to DotBot
//#include "board.h"
#include "hc_sr04.h"
#include "radio.h"


//=========================== defines =========================================

/**
 * Struct used to store internal variables
 */
typedef struct {
    NRF_TIMER_Type *timer0;                   // variable for passing the Timer for sensor trigger 
    NRF_TIMER_Type *timer1;                   // variable for passing the Timer for sensor echo measurement
} us_ranging_vars_t;

//=========================== prototypes =========================================

/**
 * callback for the US echo measurement
 */
void us_callback(uint32_t us_reading);     

/**
 * callback for timer0 after pulse_offset_ms + pulse_duration_ms
 */
void timer_callback(void);                 

/**
 * callback for the radio, called when Rx event
 */
void radio_callback(uint8_t *packet, uint8_t length);

/**
 * A function to setup the radio for the application needs. This function should be called after db_radio_init 
 */
void us_radio_setup(void);

//=========================== variables =========================================


static us_ranging_vars_t us_ranging_vars;
 

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    us_ranging_vars.timer0 = NRF_TIMER0;
    us_ranging_vars.timer1 = NRF_TIMER1;

    // uncomment if using this code on DotBot Turn ON the DotBot board regulator
    //db_board_init();

    // Initialize Radio 
    db_radio_init(&radio_callback); // Set the callback function.

    // initilize the US sensor, set callback and chose the timers
    us_init(&us_callback, &timer_callback, us_ranging_vars.timer0, us_ranging_vars.timer1);

    // Configure the radio for US ranging
    us_radio_setup();

    // start ranging
    us_start();
  
    // THIS IS ALREADY ENABLED IN db_radio_init - start high frequency clock needed for PPI
    //hfclk_init();

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
    
/**
 *  @brief This is a callback for the radio Rx event.
 *         The code ends up here every time a radio packet is received.
 *         Here we setup the new values for the compare ragisters in order to change the offset of the US trigger timer.
 *         From here the PPI for US ranging starts every time a packet is received.
 */
void radio_callback(uint8_t *packet, uint8_t length) {

    //TODO
    // stop ranging
    //us_stop();

    // start ranging
    //us_start();
        
}

/**
 *  @brief A function to setup the radio for the application needs. This function should be called after db_radio_init 
 */
void us_radio_setup(void) {

    db_radio_set_frequency(8);      // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();           // Start receiving packets.

}