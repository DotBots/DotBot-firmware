/**
 * @file timer.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "timer" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <stdbool.h>
#include <stdint.h>
#include "timer.h"

//=========================== define ===========================================

#define TIMER_RTC           (NRF_RTC1)          /**< Backend RTC peripheral used by the timer */
#define TIMER_RTC_IRQ       (RTC1_IRQn)         /**< IRQ corresponding to the RTC used */
#define TIMER_RTC_ISR       (RTC1_IRQHandler)   /**< ISR function handler corresponding to the RTC used */
#define TIMER_RTC_CB_CHANS  (3)                 /**< Number of channels that can be used for periodic callbacks */

typedef struct {
    uint32_t period_ticks;
    timer_cb_t callback;
} timer_callback_t;

typedef struct {
    timer_callback_t timer_callback[TIMER_RTC_CB_CHANS];
    bool waiting;
} timer_vars_t;

//=========================== prototypes =======================================

static uint32_t _ticks_to_ms(uint32_t ticks);
static uint32_t _ms_to_ticks(uint32_t ms);

//=========================== variables ========================================

static timer_vars_t _timer_vars;

//=========================== public ===========================================

/**
 * @brief Configure an RTC timer with millisecond precision
 */
void db_timer_init(void) {
    /* No delay is running after initialization */
    _timer_vars.waiting = false;

    // Configure and start Low Frequency clock
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;

    // Configure the RTC
    TIMER_RTC->TASKS_STOP           = 1;
    TIMER_RTC->TASKS_CLEAR          = 1;
    TIMER_RTC->PRESCALER            = 0;    // Run RTC at 32768Hz
    TIMER_RTC->EVTENSET = (1 << RTC_EVTENSET_COMPARE3_Pos);
    TIMER_RTC->INTENSET = (1 << RTC_EVTENSET_COMPARE3_Pos);
    NVIC_EnableIRQ(TIMER_RTC_IRQ);

    // Start the timer
    TIMER_RTC->TASKS_START = 1;
}

/**
 * @brief Return the number of ms elapsed since the timer started
 */
uint32_t db_timer_now(void) {
    return _ticks_to_ms(TIMER_RTC->COUNTER);
}

/**
 * @brief Set a callback to be called periodically
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        periodicity in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_periodic(uint8_t channel, uint32_t ms, timer_cb_t cb) {
    _timer_vars.timer_callback[channel].period_ticks    = _ms_to_ticks(ms);
    _timer_vars.timer_callback[channel].callback        = cb;
    TIMER_RTC->EVTENSET = (1 << (RTC_EVTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->INTENSET = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->CC[channel] = TIMER_RTC->COUNTER + _timer_vars.timer_callback[channel].period_ticks;
}

/**
 * @brief Add a delay in milliseconds
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_delay_ms(uint32_t ms) {
    TIMER_RTC->CC[3] = TIMER_RTC->COUNTER + _ms_to_ticks(ms);
    _timer_vars.waiting = true;
    while (_timer_vars.waiting) {
        __WFE();    // Let's go to sleep
    }
}

/**
 * @brief Add a delay in seconds
 *
 * @param[in] s delay in seconds
 */
void db_timer_delay_s(uint32_t s) {
    db_timer_delay_ms(s * 1000);
}

//=========================== private ==========================================

static uint32_t _ticks_to_ms(uint32_t ticks) {
    return ticks / 33;
}

static uint32_t _ms_to_ticks(uint32_t ms) {
    return ms * 33;
}

//=========================== interrupt ========================================

void TIMER_RTC_ISR(void) {
    if (TIMER_RTC->EVENTS_COMPARE[3] == 1) {
        TIMER_RTC->EVENTS_COMPARE[3] = 0;
        _timer_vars.waiting = false;
        __SEV();
    }

    for (uint8_t channel = 0; channel < TIMER_RTC_CB_CHANS; ++channel) {
        if (TIMER_RTC->EVENTS_COMPARE[channel] == 1) {
            TIMER_RTC->EVENTS_COMPARE[channel] = 0;
            TIMER_RTC->CC[channel] += _timer_vars.timer_callback[channel].period_ticks;
            if (_timer_vars.timer_callback[channel].callback) {
                _timer_vars.timer_callback[channel].callback();
            }
        }
    }
}
