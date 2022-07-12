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
#include <nrf_peripherals.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "clock.h"
#include "timer.h"

//=========================== define ===========================================

#define TIMER_RTC           (NRF_RTC2)          ///< Backend RTC peripheral used by the timer
#define TIMER_RTC_IRQ       (RTC2_IRQn)         ///< IRQ corresponding to the RTC used
#define TIMER_RTC_ISR       (RTC2_IRQHandler)   ///< ISR function handler corresponding to the RTC used
#define TIMER_RTC_CB_CHANS  (RTC2_CC_NUM - 1)   ///< Number of channels that can be used for periodic callbacks

typedef struct {
    uint32_t    period_ticks;                   ///< Period in ticks between each callback
    bool        one_shot;                       ///< Whether this is a one shot callback
    timer_cb_t  callback;                       ///< Pointer to the callback function
} timer_callback_t;

typedef struct {
    timer_callback_t    timer_callback[TIMER_RTC_CB_CHANS]; ///< List of timer callback structs
    bool                running;                            ///< Whether the delay timer is running
} timer_vars_t;

//=========================== prototypes =======================================

static uint32_t _ms_to_ticks(uint32_t ms);

//=========================== variables ========================================

static timer_vars_t _timer_vars;

//=========================== public ===========================================

/**
 * @brief Configure an RTC timer with millisecond precision
 */
void db_timer_init(void) {
    // No delay is running after initialization
    _timer_vars.running = false;

    // Configure and start Low Frequency clock
    db_lfclk_init();

    // Configure the RTC
    TIMER_RTC->TASKS_STOP   = 1;
    TIMER_RTC->TASKS_CLEAR  = 1;
    TIMER_RTC->PRESCALER    = 0;    // Run RTC at 32768Hz
    TIMER_RTC->EVTENSET     = (RTC_EVTENSET_COMPARE3_Enabled << RTC_EVTENSET_COMPARE3_Pos);
    TIMER_RTC->INTENSET     = (RTC_EVTENSET_COMPARE3_Enabled << RTC_EVTENSET_COMPARE3_Pos);
    NVIC_EnableIRQ(TIMER_RTC_IRQ);

    // Start the timer
    TIMER_RTC->TASKS_START = 1;
}

/**
 * @brief Set a callback to be called periodically
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        periodicity in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_periodic_ms(uint8_t channel, uint32_t ms, timer_cb_t cb) {
    assert(channel >= 0 && channel < TIMER_RTC_CB_CHANS);  // Make sure the required channel is correct
    assert(cb); // Make sure the callback function is valid

    _timer_vars.timer_callback[channel].period_ticks    = _ms_to_ticks(ms);
    _timer_vars.timer_callback[channel].one_shot        = false;
    _timer_vars.timer_callback[channel].callback        = cb;
    TIMER_RTC->EVTENSET = (1 << (RTC_EVTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->INTENSET = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->CC[channel] = TIMER_RTC->COUNTER + _timer_vars.timer_callback[channel].period_ticks;
}

/**
 * @brief Set a callback to be called once after an amount of ticks (1 tick ~= 30us)
 *
 * @param[in] channel   RTC channel used
 * @param[in] ticks     delay in ticks
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ticks(uint8_t channel, uint32_t ticks, timer_cb_t cb) {
    assert(channel >= 0 && channel < TIMER_RTC_CB_CHANS);  // Make sure the required channel is correct
    assert(cb); // Make sure the callback function is valid

    _timer_vars.timer_callback[channel].period_ticks    = ticks;
    _timer_vars.timer_callback[channel].one_shot        = true;
    _timer_vars.timer_callback[channel].callback        = cb;
    TIMER_RTC->EVTENSET = (1 << (RTC_EVTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->INTENSET = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    TIMER_RTC->CC[channel] = TIMER_RTC->COUNTER + _timer_vars.timer_callback[channel].period_ticks;
}

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] channel   RTC channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_ms(uint8_t channel, uint32_t ms, timer_cb_t cb) {
    db_timer_set_oneshot_ticks(channel, _ms_to_ticks(ms), cb);
}

/**
 * @brief Set a callback to be called once after an amount of seconds
 *
 * @param[in] channel   RTC channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_set_oneshot_s(uint8_t channel, uint32_t s, timer_cb_t cb) {
    db_timer_set_oneshot_ticks(channel, s * 32768, cb);
}

/**
 * @brief Add a delay in ticks (1 tick ~ 30us)
 *
 * @param[in] ticks delay in ticks
 */
void db_timer_delay_ticks(uint32_t ticks) {
    TIMER_RTC->CC[TIMER_RTC_CB_CHANS] = TIMER_RTC->COUNTER + ticks;
    _timer_vars.running = true;
    while (_timer_vars.running) {
        // Let's go to sleep
        // See https://devzone.nordicsemi.com/f/nordic-q-a/49010/methods-to-put-the-nrf52-to-sleep-in-a-spinlock-loop
        // for details
        __WFE();
        __SEV();
        __WFE();
    }
}

/**
 * @brief Add a delay in milliseconds
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_delay_ms(uint32_t ms) {
    db_timer_delay_ticks(_ms_to_ticks(ms));
}

/**
 * @brief Add a delay in seconds
 *
 * @param[in] s delay in seconds
 */
void db_timer_delay_s(uint32_t s) {
    db_timer_delay_ticks(s * 32768);
}

//=========================== private ==========================================

static uint32_t _ms_to_ticks(uint32_t ms) {
    // Return an estimation of the number of ticks matching the
    // required number of milliseconds. The estimation should be just above to
    // avoid firing the timer in advance.
    return (ms * 33) - (uint32_t)(ms * (float)(33 - (32768.0 / 1000)));
}

//=========================== interrupt ========================================

void TIMER_RTC_ISR(void) {
    if (TIMER_RTC->EVENTS_COMPARE[TIMER_RTC_CB_CHANS] == 1) {
        TIMER_RTC->EVENTS_COMPARE[TIMER_RTC_CB_CHANS] = 0;
        _timer_vars.running = false;
        __SEV();
    }

    for (uint8_t channel = 0; channel < TIMER_RTC_CB_CHANS; ++channel) {
        if (TIMER_RTC->EVENTS_COMPARE[channel] == 1) {
            TIMER_RTC->EVENTS_COMPARE[channel] = 0;
            if (_timer_vars.timer_callback[channel].one_shot) {
                TIMER_RTC->EVTENCLR = (1 << (RTC_EVTENCLR_COMPARE0_Pos + channel));
                TIMER_RTC->INTENCLR = (1 << (RTC_INTENCLR_COMPARE0_Pos + channel));
            }
            else {
                TIMER_RTC->CC[channel] += _timer_vars.timer_callback[channel].period_ticks;
            }
            if (_timer_vars.timer_callback[channel].callback) {
                _timer_vars.timer_callback[channel].callback();
            }
        }
    }
}
