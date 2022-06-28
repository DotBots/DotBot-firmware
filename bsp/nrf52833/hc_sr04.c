/**
 * @file us_ranging.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "us_ranging" bsp module.
 *
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @copyright Inria, 2022
 */
 
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hc_sr04.h"

//=========================== define ==========================================

// GPIOTE for turning ON US sensor and READING the range measurement 
#define US_ON_PORT              1UL        // output port number
#define US_ON_PIN               10UL       // output pin number
#define US_READ_PORT            1UL        
#define US_READ_PIN             6UL

#define US_ON_CH                0UL
#define US_READ_CH_LoToHi       1UL
#define US_READ_CH_HiToLo       2UL

// Timer0 interrupt prioprity
#define TIMER0_INT_PRIORITY     2

//=========================== prototypes =======================================

void hc_sr04_gpio(void);
void hc_sr04_timers(void);

//=========================== variables =======================================

typedef struct {
    us_callback_t     us_callback;        // Function pointer, stores the callback to use in the GPIOTE_IRQn handler.
    timer_callback_t  timer_callback;     // Function pointer, stores the callback to use in the GPIOTE_IRQn handler. 
   
    NRF_TIMER_Type    *us_on_timer;       // Pointer to the TIMER structure used for triggering US sensor
    NRF_TIMER_Type    *us_read_timer;     // Pointer to the TIMER structure used for reading the range on the US sensor

    volatile uint32_t us_reading;         // Variable to store pulse duration of the US echo pin
} us_vars_t;

static us_vars_t us_vars;

//=========================== public ==========================================
 
/**
 * @brief Function for initializing variables and calling private functions for using the hc_sr04 US sensor
 */
void hc_sr04_init(us_callback_t us_callback, timer_callback_t timer_callback, NRF_TIMER_Type *us_on, NRF_TIMER_Type *us_read) {
    
    // Assign the callback function that will be called when ranging is performed.
    us_vars.us_callback = us_callback;

    // Assign the callback function that will be called when compare 0 is reached.
    us_vars.timer_callback = timer_callback;

    // Assign the Timers for turning ON the US sensor and for US sensor readings
    us_vars.us_on_timer   = us_on;
    us_vars.us_read_timer = us_read;
    
    // initialize GPIOTE for US on and US read
    hc_sr04_gpio();

    // initialize timers for US on and US read
    hc_sr04_timers();
}

/**
 * @brief This function is public and it sets the PPI channels to allow the US trigger and US echo. Ranging starts by calling this function
 */
void hc_sr04_ppi_setup(void) {  
    // get endpoint addresses
    uint32_t us_power_task_addr             = (uint32_t)&NRF_GPIOTE->TASKS_OUT[US_ON_CH];

    uint32_t us_read_event_low_high_addr    = (uint32_t)&NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi];
    uint32_t us_read_event_high_low_addr    = (uint32_t)&NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo];

    uint32_t timer3_task_start_addr         = (uint32_t)&us_vars.us_read_timer->TASKS_START;
    uint32_t timer3_task_clear_addr         = (uint32_t)&us_vars.us_read_timer->TASKS_CLEAR;
    uint32_t timer3_task_stop_addr          = (uint32_t)&us_vars.us_read_timer->TASKS_STOP;
    uint32_t timer3_task_capture_addr       = (uint32_t)&us_vars.us_read_timer->TASKS_CAPTURE[0];
    
    uint32_t timer2_events_compare_0_addr   = (uint32_t)&us_vars.us_on_timer->EVENTS_COMPARE[0];
    uint32_t timer2_events_compare_1_addr   = (uint32_t)&us_vars.us_on_timer->EVENTS_COMPARE[1];

    // set endpoints
    NRF_PPI->CH[0].EEP       = timer2_events_compare_0_addr;
    NRF_PPI->CH[0].TEP       = us_power_task_addr;

    NRF_PPI->CH[1].EEP       = timer2_events_compare_1_addr;
    NRF_PPI->CH[1].TEP       = us_power_task_addr;

    NRF_PPI->CH[2].EEP       = us_read_event_low_high_addr;
    NRF_PPI->CH[2].TEP       = timer3_task_clear_addr;
    NRF_PPI->FORK[2].TEP     = timer3_task_start_addr;

    NRF_PPI->CH[3].EEP       = us_read_event_high_low_addr;
    NRF_PPI->CH[3].TEP       = timer3_task_capture_addr;
    NRF_PPI->FORK[3].TEP     = timer3_task_stop_addr;

    // enable channels
    NRF_PPI->CHENSET = (PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos) | 
                       (PPI_CHENSET_CH1_Enabled << PPI_CHENSET_CH1_Pos) |
                       (PPI_CHENSET_CH2_Enabled << PPI_CHENSET_CH2_Pos) |
                       (PPI_CHENSET_CH3_Enabled << PPI_CHENSET_CH3_Pos);                                      
                     
}

void hc_sr04_start(void) {
    
    // Clear timer
    us_vars.us_on_timer->TASKS_CLEAR = (TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos);
    /* Start the US trigger timer. 
       From here the PPI chain for US ranging starts */
    us_vars.us_on_timer->TASKS_START = (TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos);

}

/**
 * @brief This function is public and it disables the PPI channels to stop the US trigger and ranging.
 */
void hc_sr04_stop(void) {

    // Stop the US ON timer
    us_vars.us_on_timer->TASKS_STOP = (TIMER_TASKS_STOP_TASKS_STOP_Trigger << TIMER_TASKS_STOP_TASKS_STOP_Pos);

    // disable PPI channels for US ranging
    NRF_PPI->CHENCLR = (PPI_CHENCLR_CH0_Enabled << PPI_CHENSET_CH0_Pos) | 
                       (PPI_CHENCLR_CH1_Enabled << PPI_CHENSET_CH1_Pos) |
                       (PPI_CHENCLR_CH2_Enabled << PPI_CHENSET_CH2_Pos) |
                       (PPI_CHENCLR_CH3_Enabled << PPI_CHENSET_CH3_Pos);
}

/**
 * @brief This function is public and it sets the compare registers for US trigger timer
 */
void hc_sr04_on_set_trigger(double duration_ms, double offset_ms) {

    // Set compare values for US on
    us_vars.us_on_timer->CC[0]   = offset_ms                   * 1000;  // first compare register for setting the offset and start of the pulse
    us_vars.us_on_timer->CC[1]   = (offset_ms + duration_ms)   * 1000;  // second compare for pulse duration

}

/**
 * @brief This function is public and it enables HFCLK
 */
void hfclk_init(void) {   
    // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;    // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;    // Start the clock
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {;} // Wait for the clock to actually start.
}

//=========================== private ==========================================

/**
 * @brief Function for initializing GPIOTE for US trigger and US echo signals.
 */
void hc_sr04_gpio(void) {
    //configure US_ON as an output PIN which toggles 
    NRF_GPIOTE->CONFIG[US_ON_CH]          = (GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos)       |
                                            (US_ON_PIN                     << GPIOTE_CONFIG_PSEL_Pos)       |
                                            (US_ON_PORT                    << GPIOTE_CONFIG_PORT_Pos)       |
                                            (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)   |
                                            (GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos);
   
    // configure the US_READ as input PIN detecting low to high edge of the signal
    NRF_GPIOTE->CONFIG[US_READ_CH_LoToHi] = (GPIOTE_CONFIG_MODE_Event        << GPIOTE_CONFIG_MODE_Pos)     |
                                            (US_READ_PIN                     << GPIOTE_CONFIG_PSEL_Pos)     |
                                            (US_READ_PORT                    << GPIOTE_CONFIG_PORT_Pos)     |
                                            (GPIOTE_CONFIG_POLARITY_LoToHi   << GPIOTE_CONFIG_POLARITY_Pos);

    // configure the US_READ as input PIN detecting high to low edge of the signal
    NRF_GPIOTE->CONFIG[US_READ_CH_HiToLo] = (GPIOTE_CONFIG_MODE_Event        << GPIOTE_CONFIG_MODE_Pos)     |
                                            (US_READ_PIN                     << GPIOTE_CONFIG_PSEL_Pos)     |
                                            (US_READ_PORT                    << GPIOTE_CONFIG_PORT_Pos)     |
                                            (GPIOTE_CONFIG_POLARITY_HiToLo   << GPIOTE_CONFIG_POLARITY_Pos);
     
    // Configure the Interruptions
    NVIC_DisableIRQ(GPIOTE_IRQn); 

    // Enable interrupt for the HiToLo edge of the input
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos); 

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);    // Clear the flag for any pending radio interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief This function is private and it sets up the Bitmode and Prescaler for US trigger and US echo timers
 */
void hc_sr04_timers(void) {
    // configure the Timer for US on
    us_vars.us_on_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    us_vars.us_on_timer->PRESCALER = 4UL;

    // configure the Timer for US read
    us_vars.us_read_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    us_vars.us_read_timer->PRESCALER = 4UL;

    // set ON Timer to clear after the trigger pulse 
    us_vars.us_on_timer->SHORTS  = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);

    // Configure the Interruptions for the Timer0 used as the ON US sensor timer
    NVIC_DisableIRQ(TIMER0_IRQn); 

    // enable interrupts on Compare 1
    us_vars.us_on_timer->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);

    NVIC_SetPriority(TIMER0_IRQn, TIMER0_INT_PRIORITY);
        
    NVIC_ClearPendingIRQ(TIMER0_IRQn);    // Clear the flag for any pending radio interrupt
    // enable interupts
    NVIC_EnableIRQ(TIMER0_IRQn);
}

//=========================== interrupt handlers ==============================

/**
 * @brief ISR for reading ranging value from the sensor. It captures the value of the Timer1 (pulse width) and calls the callback for US reading
 * 
 */
void GPIOTE_IRQHandler(void){
  
    if(NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] != 0)
    {
        NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] = 0UL;

        // range value captured trough PPI
        us_vars.us_reading = us_vars.us_read_timer->CC[0];

        // Call callback defined by user.
        (*us_vars.us_callback)(us_vars.us_reading);
    }
}

/**
* brief ISR for TIMER0 which is in charge of controling the US sensor trigger pin
*/
void TIMER0_IRQHandler(void){

    if((NRF_TIMER0->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {   
        NRF_TIMER0->EVENTS_COMPARE[1] = 0; //Clear compare register 0 event	 
        
        // Call callback defined by user.
        (*us_vars.timer_callback)();      
    }
}
