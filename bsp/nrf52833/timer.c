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
#define TIMER_RTC_CC_SIZE   (4)

typedef struct {
    uint32_t period_ms;
    timer_cb_t callback;
} timer_vars_t;

//=========================== variables ========================================

static timer_vars_t _timer_vars[TIMER_RTC_CC_SIZE];

//=========================== public ===========================================

/**
 * @brief Configure an RTC timer with millisecond precision
 */
void db_timer_init(void) {
    // Configure and start Low Frequency clock
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;

    // Configure the RTC
    TIMER_RTC->TASKS_STOP           = 1;
    TIMER_RTC->TASKS_CLEAR          = 1;
    TIMER_RTC->PRESCALER            = 32;    // Run RTC with millisecond precision
    NVIC_EnableIRQ(TIMER_RTC_IRQ);

    // Start the timer
    TIMER_RTC->TASKS_START = 1;
}

/**
 * @brief Return the number of ms elapsed since the timer started
 */
uint32_t db_timer_now(void) {
    return TIMER_RTC->COUNTER;
}

/**
 * @brief Set a callback to be called periodically
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        periodicity in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_periodic(uint8_t channel, uint32_t ms, timer_cb_t cb) {
    _timer_vars[channel].period_ms  = ms;
    _timer_vars[channel].callback   = cb;
    TIMER_RTC->EVTENSET = (1 << (RTC_EVTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->INTENSET = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->CC[channel] = TIMER_RTC->COUNTER + _timer_vars[channel].period_ms;
}

/**
 * @brief Add a delay in milliseconds
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_delay_ms(uint32_t ms) {
    uint32_t target     = TIMER_RTC->COUNTER + ms;
    TIMER_RTC->EVTENSET = (RTC_EVTENSET_TICK_Enabled << RTC_EVTENSET_TICK_Pos);
    TIMER_RTC->INTENSET = (RTC_INTENSET_TICK_Enabled << RTC_INTENSET_TICK_Pos);
    while (TIMER_RTC->COUNTER != target) {
        __WFI();    // Let's go to sleep
    }
    TIMER_RTC->EVTENCLR = (RTC_EVTENCLR_TICK_Enabled << RTC_EVTENCLR_TICK_Pos);
    TIMER_RTC->INTENCLR = (RTC_INTENCLR_TICK_Enabled << RTC_INTENCLR_TICK_Pos);
}

/**
 * @brief Add a delay in seconds
 *
 * @param[in] s delay in seconds
 */
void db_timer_delay_s(uint32_t s) {
    db_timer_delay_ms(s * 1000);
}

//=========================== interrupt ========================================

void TIMER_RTC_ISR(void) {
    if (TIMER_RTC->EVENTS_TICK == 1) {
        TIMER_RTC->EVENTS_TICK = 0;
    }

    for (uint8_t channel = 0; channel < TIMER_RTC_CC_SIZE; ++channel) {
        if (TIMER_RTC->EVENTS_COMPARE[channel] == 1) {
            TIMER_RTC->EVENTS_COMPARE[channel] = 0;
            TIMER_RTC->CC[channel] += _timer_vars[channel].period_ms;
            if (_timer_vars[channel].callback) {
                _timer_vars[channel].callback();
            }
        }
    }
}
