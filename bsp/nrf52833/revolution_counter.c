/**
 * @file board.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "board" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "revolution_counter.h"

//=========================== defines =========================================

#ifndef ME_LEFT_PIN
#define ME_LEFT_PIN           (17)
#endif

#ifndef ME_LEFT_TIMER
#define ME_LEFT_TIMER         (NRF_TIMER0)
#endif

#ifndef ME_LEFT_PPI_CHAN
#define ME_LEFT_PPI_CHAN      (0)
#endif

#ifndef ME_LEFT_GPIOTE_CHAN
#define ME_LEFT_GPIOTE_CHAN   (0)
#endif

#ifndef ME_RIGHT_PIN
#define ME_RIGHT_PIN          (15)
#endif

#ifndef ME_RIGHT_TIMER
#define ME_RIGHT_TIMER        (NRF_TIMER1)
#endif

#ifndef ME_RIGHT_PPI_CHAN
#define ME_RIGHT_PPI_CHAN     (1)
#endif

#ifndef ME_RIGHT_GPIOTE_CHAN
#define ME_RIGHT_GPIOTE_CHAN  (1)
#endif

#ifndef ME_RTC
#define ME_RTC                (NRF_RTC0)
#endif

#ifndef ME_RTC_IRQ
#define ME_RTC_IRQ            (RTC0_IRQn)
#endif

#ifndef ME_RTC_ISR
#define ME_RTC_ISR            (RTC0_IRQHandler)
#endif

/*
 * number of cycles captured within last 10ms (one rotation is 3.77mm distance of the wheel)
 * so speed = (3.7 * 0.001 * cycles / (10 * 0.01) m/s
 */
#define ME_CYCLES_TO_SPEED(cycles)  (3.77 * 0.01 * cycles)

/*
 * 1 cycle corresponds to one rotation, so convert to the number of minutes, given the timer frequency
 */
#define ME_CYCLES_TO_RPM(cycles)    (600 * cycles)


//=========================== variables =========================================

/*
 * Global variables used to store cycle counts at the beginning and at the end of
 * an RTC timeframe, for each side
 */
static uint32_t last_left_counts = 0;
static uint32_t previous_left_counts = 0;
static uint32_t last_right_counts = 0;
static uint32_t previous_right_counts = 0;

//=========================== public ==========================================

/**
 * @brief Initalize the revolution counter driver
 *
 * 2 GPIOTE input pins with one timer each. Each time a magnet comes in front of the magnetic encoder,
 * GPIOTE event is triggered which clears the timer ticks counter, for each side (left and right).
 * The speed/rpm are computed by the user code on demand by capturing the timer current count, reading
 * the timer CC register and clearing the timer count. Computations are done with the `ME_TICK_TO_*`
 * constants.
 */
void db_revolution_counter_init(void) {
    /* Configure pin connected to left magnetic encoder sensor, input pullup */
    NRF_P0->PIN_CNF[ME_LEFT_PIN] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[ME_LEFT_PIN] &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[ME_LEFT_GPIOTE_CHAN] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                              (ME_LEFT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                              (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    /* Configure pin connected to right magnetic encoder sensor, input pullup */
    NRF_P0->PIN_CNF[ME_RIGHT_PIN] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[ME_RIGHT_PIN] &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[ME_RIGHT_GPIOTE_CHAN] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                               (ME_RIGHT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                               (GPIOTE_CONFIG_POLARITY_LoToHi  << GPIOTE_CONFIG_POLARITY_Pos);

    /* Configure and clear timers */
    ME_LEFT_TIMER->MODE = TIMER_MODE_MODE_LowPowerCounter;
    ME_LEFT_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    ME_LEFT_TIMER->INTENCLR = 1;
    ME_LEFT_TIMER->TASKS_CLEAR = 1;

    ME_RIGHT_TIMER->MODE = TIMER_MODE_MODE_LowPowerCounter;
    ME_RIGHT_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    ME_RIGHT_TIMER->INTENCLR = 1;
    ME_RIGHT_TIMER->TASKS_CLEAR = 1;

    /* Configure gpiote/timer count PPI
       - PO.17 (left magnetic encoder) event connected to PPI channel 0 which fires Timer 0 capture 0
       - PO.15 (right magnetic encoder) event connected to PPI channel 1 which fires Timer 0 capture 1
    */
    NRF_PPI->CH[ME_LEFT_PPI_CHAN].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[ME_LEFT_GPIOTE_CHAN];
    NRF_PPI->CH[ME_LEFT_PPI_CHAN].TEP = (uint32_t)&ME_LEFT_TIMER->TASKS_COUNT;
    NRF_PPI->CH[ME_RIGHT_PPI_CHAN].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[ME_RIGHT_GPIOTE_CHAN];
    NRF_PPI->CH[ME_RIGHT_PPI_CHAN].TEP = (uint32_t)&ME_RIGHT_TIMER->TASKS_COUNT;

    /* Enable PPI channels */
    NRF_PPI->CHENSET = (1 << ME_RIGHT_PPI_CHAN) | (1 << ME_LEFT_PPI_CHAN);

    /* Configure RTC with 10ms fire event delay */
    ME_RTC->PRESCALER = 327;
    ME_RTC->INTENSET = RTC_INTENSET_TICK_Enabled;
    NVIC_EnableIRQ(ME_RTC_IRQ);
    /* Start RTC */
    ME_RTC->TASKS_START = 1;
}


void ME_RTC_ISR(void) {
    if (ME_RTC->EVENTS_TICK == 1) {
        NVIC_ClearPendingIRQ(ME_RTC_IRQ);
        ME_RTC->EVENTS_TICK = 0;
        ME_LEFT_TIMER->TASKS_CAPTURE[0] = 1;
        ME_RIGHT_TIMER->TASKS_CAPTURE[0] = 1;
        previous_left_counts = last_left_counts;
        previous_right_counts = last_right_counts;
        last_left_counts = ME_LEFT_TIMER->CC[0];
        last_right_counts = ME_RIGHT_TIMER->CC[0];
    }
}

static inline uint32_t _db_left_cycles(void) {
    if (last_left_counts < previous_left_counts) {
        return UINT32_MAX - previous_left_counts + last_left_counts;
    }
    return last_left_counts - previous_left_counts;
}

static inline uint32_t _db_right_cycles(void) {
    if (last_right_counts < previous_right_counts) {
        return UINT32_MAX - previous_right_counts + last_right_counts;
    }
    return last_right_counts - previous_right_counts;
}

void db_board_encoder_timers_start(void) {
    ME_LEFT_TIMER->TASKS_START = 1;
    ME_RIGHT_TIMER->TASKS_START = 1;
}

void db_board_encoder_timers_stop(void) {
    ME_LEFT_TIMER->TASKS_STOP = 1;
    ME_RIGHT_TIMER->TASKS_STOP = 1;
}

uint32_t db_board_get_left_rpm(void) {
    return ME_CYCLES_TO_RPM(_db_left_cycles());
}

uint32_t db_board_get_right_rpm(void) {
    return ME_CYCLES_TO_RPM(_db_right_cycles());
}

float db_board_get_left_speed(void) {
    return ME_CYCLES_TO_SPEED(_db_left_cycles()); /* m/s */
}

float db_board_get_right_speed(void) {
    return ME_CYCLES_TO_SPEED(_db_right_cycles()); /* m/s */
}
