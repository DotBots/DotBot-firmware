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
#include "rgbled.h"

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

typedef enum {
    MOVE_RAW = 0,
    RGB_LED = 2,
} joystick_command_type_t;

typedef struct __attribute__((packed)) {
    int8_t left_x;
    int8_t left_y;
    int8_t right_x;
    int8_t right_y;
} joystick_command_t;

typedef struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgbled_command_t;

static joystick_command_t *command;

//=========================== variables =========================================

//=========================== main =========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    TIMEOUT_RTC->TASKS_CLEAR = 1; /* Clear RTC counter */
    /* parse received packet and update the motors' speeds */
    if (pkt[0] != 0) {
        /* Only version 0 is supported */
        return;
    }

    switch (pkt[1]) {
        case MOVE_RAW:
        {
            joystick_command_t *command = (joystick_command_t *)&pkt[2];
            int16_t left = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t right = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            db_motors_setSpeed(left, right);
        }
            break;
        case RGB_LED:
        {
            rgbled_command_t *command = (rgbled_command_t *)&pkt[2];
            printf("%d-%d-%d\n", command->r, command->g, command->b
            );
            db_rgbled_set(command->r, command->g, command->b);
        }
            break;
    }
}

static void db_timeout_rtc_init(void) {
    TIMEOUT_RTC->TASKS_STOP = 1;
    TIMEOUT_RTC->TASKS_CLEAR = 1;
    /* Configure RTC with 125ms delay between ticks */
    TIMEOUT_RTC->PRESCALER = (uint32_t)((1 << 12) - 1);
    TIMEOUT_RTC->INTENSET = RTC_INTENSET_TICK_Enabled;
    TIMEOUT_RTC->EVTENSET = RTC_EVTENSET_TICK_Enabled;
    NVIC_EnableIRQ(TIMEOUT_RTC_IRQ);
    /* Start RTC */
    TIMEOUT_RTC->TASKS_START = 1;
}

void TIMEOUT_RTC_ISR(void) {
    NVIC_ClearPendingIRQ(TIMEOUT_RTC_IRQ);
    if (TIMEOUT_RTC->EVENTS_TICK) {
        TIMEOUT_RTC->EVENTS_TICK = 0;
        /* Stop the motors if the RTC fires a compare events, e.g. when no packet was received during 100ms */
        db_motors_setSpeed(0, 0);
    }
}

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    puts("DotBot joystick");
    db_board_init();
    db_rgbled_init();
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
