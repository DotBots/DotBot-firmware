/**
 * @file 03app_dotbot_remote_control.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to control a single dotbot remotely with a joystick using radio.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "motors.h"
#include "radio.h"

//=========================== defines =========================================

#ifndef TIMEOUT_RTC
#define TIMEOUT_RTC           (NRF_RTC1)
#endif

#ifndef TIMEOUT_RTC_IRQ
#define TIMEOUT_RTC_IRQ       (RTC1_IRQn)
#endif

#ifndef TIMEOUT_RTC_ISR
#define TIMEOUT_RTC_ISR       (RTC1_IRQHandler)
#endif

//=========================== variables =========================================

//=========================== main =========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    TIMEOUT_RTC->TASKS_CLEAR = 1; /* Clear RTC counter */

    /* TODO: parse received packet and update the motors' speeds */
}

static void db_timeout_rtc_init(void) {
    /* Configure RTC with 10ms delay between ticks */
    TIMEOUT_RTC->PRESCALER = 327;
    TIMEOUT_RTC->CC[0] = 10;  /* Fires a compare0 event after 100ms */
    TIMEOUT_RTC->INTENSET = RTC_INTENSET_COMPARE0_Enabled;
    NVIC_EnableIRQ(TIMEOUT_RTC_IRQ);
    /* Start RTC */
    TIMEOUT_RTC->TASKS_START = 1;
}

void TIMEOUT_RTC_ISR(void) {
    if (TIMEOUT_RTC->EVENTS_COMPARE[0]) {
        NVIC_ClearPendingIRQ(TIMEOUT_RTC_IRQ);
        TIMEOUT_RTC->EVENTS_COMPARE[0] = 0;
        /* Stop the motors if the RTC fires a compare events, e.g. when no packet was received during 100ms */
        db_motors_setSpeed(0, 0);
    }
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {

    db_board_init();
    db_motors_init();
    db_radio_init(&radio_callback);
    db_radio_set_frequency(8);      // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();           // Start receiving packets.
    db_timeout_rtc_init();          // Start timeout RTC used to check packet reception

    while (1) {
        __WFE(); // Enter a low power state while waiting.
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
