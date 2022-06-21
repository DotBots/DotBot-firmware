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
#include <string.h>

#include "board.h"
#include "hc_sr04.h"
#include "radio.h"

//=========================== defines ===========================================

// Defines for setting up the trigger pulse width and frequency
#define PULSE_DURATION_MS       0.01
#define PULSE_OFFSET_MS         200

// Defines for the Radio
#define NUMBER_OF_BYTES_IN_PACKET 32

//=========================== prototypes =========================================

void us_callback(uint32_t us_reading);     // callback for the US echo measurement
void timer_callback(void);                 // callback for timer0 after pulse_offset_ms + pulse_duration_ms

/**
 * callback for the radio, called when Rx event
 */
void radio_callback(uint8_t *packet, uint8_t length);

//=========================== variables =========================================

typedef struct {
    NRF_TIMER_Type    *timer0;                     // Pointer to the TIMER structure used for triggering US sensor
    NRF_TIMER_Type    *timer1;                     // Pointer to the TIMER structure used for reading the range on the US sensor

    uint8_t packet_tx[NUMBER_OF_BYTES_IN_PACKET];  // Radio packet Tx
    uint8_t packet_rx[NUMBER_OF_BYTES_IN_PACKET];  // Radio packet Rx
} app_vars_t;

static app_vars_t app_vars;

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    memset(&app_vars, 0, sizeof(app_vars_t));

    app_vars.timer0 = NRF_TIMER0;
    app_vars.timer1 = NRF_TIMER1;

    // uncomment if using this code on DotBot Turn ON the DotBot board regulator
    db_board_init();

    //=========================== Configure Radio =========================================

    db_radio_init_lr(&radio_callback);                  // Set radio callback and initialize Long Range BLE
    db_radio_set_frequency(8);                          // Set the RX frquency to 2408 MHz.

    db_radio_tx(app_vars.packet_tx, NUMBER_OF_BYTES_IN_PACKET);
    db_radio_rx_enable();                               // Start receiving packets.

    //=========================== Configure HC_SR04 =========================================

    // initilize the US sensor, set callback and chose the timers
    hc_sr04_init(&us_callback, &timer_callback, app_vars.timer0, app_vars.timer1);

    // initial timer settings for the trigger pulse 
    hc_sr04_on_set_trigger(PULSE_DURATION_MS, PULSE_OFFSET_MS);
    
    // start high frequency clock needed for PPI
    hfclk_init();
     
    // start ranging
    hc_sr04_start();

    while (1) {        
        __WFE();      
        __SEV();
        __WFE();
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions =========================================

/**
 *  @brief Callback function to process received packets
 *
 * This function gets called each time a packet is received.
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 * 
 */
void radio_callback(uint8_t *packet, uint8_t length) {

    // stop ranging
    hc_sr04_stop();
         
    // start ranging
    hc_sr04_start();
    
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
