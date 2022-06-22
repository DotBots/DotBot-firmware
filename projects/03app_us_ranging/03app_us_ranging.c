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
#define PULSE_DURATION_MS           0.01
#define PULSE_OFFSET_MS             200

// Defines for the Radio
#define NUMBER_OF_BYTES_IN_PACKET   32

// First byte in the Gateway message which correspond to the US ranging app type
#define ID_RANGE_ALL                0x11
#define ID_RANGE_PAIR               0x22
#define ID_RANGE_NONE               0xff

// Robot ID will be used as a uniqe identifier or Address of a robot
#define ROBOT_ID                    0x01
// Gateway ID will be used as a uniqe identifier or Address of a robot
#define GATEWAY_ID                  0x77

//=========================== prototypes =========================================

void us_callback(uint32_t us_reading);     // callback for the US echo measurement
void timer_callback(void);                 // callback for timer0 after pulse_offset_ms + pulse_duration_ms

/**
 * callback for the radio, called when Rx event
 */
void radio_callback(uint8_t *packet, uint8_t length);

//=========================== variables =========================================

enum {
    APP_FLAG_RADIO_RX    = 0x01,                                // Flag for indicating end of frame
    APP_FLAG_RTC         = 0x02,                                // Flag for indicating RTC timer event
    APP_FLAG_US_DONE     = 0x04,                                // Flag to indicate that a new US ranging is done
} app_flags_t;

typedef enum {
    RADIO_Tx             = 0x01,                                // To know when the Radio is sending the packet
    RADIO_Rx             = 0x02,                                // To know when the Radio is receiving the packet
    RADIO_Idle           = 0x04,                                // To know when the Radio is Idle
} app_radio_mode_t;

typedef enum {
    RANGE_ALL            = 0x01,                                // State that represents the experiment where all robots range at a time
    RANGE_PAIR           = 0x02,                                // State that represents the experiment where two robots range at a time
    RANGE_NONE           = 0x04,                                // State where no ranging measurements are performed
} app_state_t;

typedef struct {
    NRF_TIMER_Type       *timer0;                               // Pointer to the TIMER structure used for triggering US sensor
    NRF_TIMER_Type       *timer1;                               // Pointer to the TIMER structure used for reading the range on the US sensor

    double               trigger_duration_ms;
    uint32_t             trigger_offset_ms;
    
    app_state_t          state;                                 // Variable for storing the state of the ranging app

    volatile uint8_t     flag;                                  // Stores the app flags 

    uint8_t              packet_tx[NUMBER_OF_BYTES_IN_PACKET];  // Radio packet Tx
    uint8_t              packet_rx[NUMBER_OF_BYTES_IN_PACKET];  // Radio packet Rx
    app_radio_mode_t     radio_mode;                            // Stores the current Radio mode

} app_vars_t;

static app_vars_t app_vars;

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    
    // For debug Set the test pin (P0.31) as an output
    NRF_P0->PIN_CNF[31] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                          (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.

    memset(&app_vars, 0, sizeof(app_vars_t));

    // Initialize timers for US ON and US READ
    app_vars.timer0     = NRF_TIMER0;
    app_vars.timer1     = NRF_TIMER1;

    app_vars.trigger_duration_ms = PULSE_DURATION_MS;
    app_vars.trigger_offset_ms   = PULSE_OFFSET_MS;

    app_vars.state      = RANGE_NONE;

    // uncomment if using this code on DotBot Turn ON the DotBot board regulator
    db_board_init();

    //=========================== Configure Radio =========================================

    db_radio_init_lr(&radio_callback);                  // Set radio callback and initialize Long Range BLE
    db_radio_set_frequency(8);                          // Set the RX frquency to 2408 MHz.

    db_radio_rx_enable();                               // Start receiving packets.
    
    app_vars.radio_mode = RADIO_Rx;                     // RX mode by default

    //=========================== Configure HC_SR04 =========================================

    // initilize the US sensor, set callback and choose the timers
    hc_sr04_init(&us_callback, &timer_callback, app_vars.timer0, app_vars.timer1);

    while (1) {        

        // Board sleep if no event flag is set
        while (app_vars.flag == 0x00) {
            // Let's go to sleep
            // See https://devzone.nordicsemi.com/f/nordic-q-a/49010/methods-to-put-the-nrf52-to-sleep-in-a-spinlock-loop
            // for details
            __WFE();
            __SEV();
            __WFE();
        }

        //TODO disable int
        // handle and clear the event flags       
        switch(app_vars.state) {
            /* 
             * RANGE_NONE is the mode of the app where the 
             * gateway sent instruction to stop the ranging 
             */
            case RANGE_NONE: 
                if ((app_vars.flag == APP_FLAG_RADIO_RX) && (app_vars.radio_mode == RADIO_Rx)) {
                    // stop ranging
                    hc_sr04_stop();
                }
            break;
            /* 
             * RANGE_ALL is the mode of the app where the gateway allows the 
             * ranging of all robots at the same time
             * with given offset
             */
            case RANGE_ALL:
                switch (app_vars.flag) {                   
                    case APP_FLAG_RADIO_RX:                   // check if the Robot received a packet from the Gateway
                        if (app_vars.radio_mode == RADIO_Rx) {                     
                            db_radio_rx_disable();            // Disable the radio                        
                            app_vars.radio_mode = RADIO_Idle; // Set the radio mode to Idle
                        }                        
                    break;
                    case APP_FLAG_US_DONE:                                          // check if we have a new US reading 
                        //TODO START RTC
                        app_vars.radio_mode = RADIO_Tx;                             // set the Radio mode to Tx
                        
                        hc_sr04_stop();                                             // stop the ranging we already have a measurement
                        
                        db_radio_tx(app_vars.packet_tx, NUMBER_OF_BYTES_IN_PACKET); // send the US reading with ROBOT ID to Gateway                        
                        memset(app_vars.packet_tx, 0, sizeof(app_vars.packet_tx));  // clear the buffer

                        db_radio_rx_enable();                                       // enable the Radio packet reception
                        app_vars.radio_mode = RADIO_Rx;                             // set the Radio mode to Rx

                    break;
                    case APP_FLAG_RTC:                                              // ready to send a packet with US measurement 
                        if (app_vars.radio_mode == RADIO_Tx) {
                            //db_radio_tx(app_vars.packet_tx, NUMBER_OF_BYTES_IN_PACKET);
                            app_vars.radio_mode = RADIO_Rx;
                            //TODO
                        }
                    break;
                    default:
                    break;
                } 
                //TODO
            break; // RANGE ALL case break
            /* 
             * RANGE_PAIR is the mode of the app where the gateway allows the 
             * ranging of pairs of robots at different time slots
             */
            case RANGE_PAIR: 
                //TODO
            break; // RANGE PAIR case break
            
            default: 
                // stop ranging
                hc_sr04_stop();
            break;       
        }
        // clear flag
        app_vars.flag = 0x00;     
        //TODO enable int
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
    // check if the packet is received and if it is from the Gateway
    if((app_vars.radio_mode == RADIO_Rx) && (packet[0] == GATEWAY_ID)) {

        // set the flag for the packet reception
        app_vars.flag = APP_FLAG_RADIO_RX;   
        
        // check the mode of operation sent by the Gateway
        switch (packet[1]) {
            case ID_RANGE_NONE:
                app_vars.state = RANGE_NONE;
            break;    
            case ID_RANGE_ALL:
                app_vars.state = RANGE_ALL;
            break;
            case ID_RANGE_PAIR:
                app_vars.state = RANGE_PAIR;
            break;
            default:
                app_vars.state = RANGE_NONE;
            break;
        }
        
        if (app_vars.state == RANGE_ALL) {

            app_vars.trigger_offset_ms  = packet[2] ;
            app_vars.trigger_offset_ms |= packet[3] << 8;
            app_vars.trigger_offset_ms |= packet[4] << 16;
            app_vars.trigger_offset_ms |= packet[5] << 24;     

            hc_sr04_on_set_trigger(app_vars.trigger_duration_ms, (double)app_vars.trigger_offset_ms);

            hc_sr04_start();
            NRF_P0->OUTSET = 1 << GPIO_OUTSET_PIN31_Pos; // ranging started set the pin HIGH.
        }
        else if (app_vars.state == RANGE_PAIR) {
            //TODO
        }
    }   
}

/**
 *  @brief This is a callback for US reading.
 *         The code ends up here every time a new measurement is taken. The new measurement is stored in us_reading argument
 */
void us_callback(uint32_t us_reading) {

    // set the flag indicating that we got a new US measurement
    app_vars.flag = APP_FLAG_US_DONE; 
    
    // prepare the packet to send to a gateway
    app_vars.packet_tx[0] = ROBOT_ID;
    
    app_vars.packet_tx[1] = us_reading;
    app_vars.packet_tx[2] = us_reading >> 8;
    app_vars.packet_tx[3] = us_reading >> 16;
    app_vars.packet_tx[4] = us_reading >> 24;

    app_vars.packet_tx[5] = 0xff;

    app_vars.packet_tx[6] = '\r';
    app_vars.packet_tx[7] = '\n';
}

/**
 *  @brief This is a callback for the US trigger timer.
 *         The code ends up here every pulse_offset_ms + pulse_duration_ms times.
 *         We could change the pulse_offset_ms here by writting a new value to timer0->CC[0] and timer0->CC[1]
 */
void timer_callback(void) {
    NRF_P0->OUTCLR = 1 << GPIO_OUTCLR_PIN31_Pos;  // US trigger pulse finish set the pin LOW.
}
