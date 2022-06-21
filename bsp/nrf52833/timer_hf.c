/**
 * @file timer_hf.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "timer hf" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "clock.h"
#include "timer_hf.h"

//=========================== define ===========================================

#define TIMER_HF            (NRF_TIMER4)        /**< Backend TIMER peripheral used by the timer */
#define TIMER_HF_IRQ        (TIMER4_IRQn)       /**< IRQ corresponding to the TIMER used */
#define TIMER_HF_ISR        (TIMER4_IRQHandler) /**< ISR function handler corresponding to the TIMER used */
#define TIMER_HF_CB_CHANS   (5)                 /**< Number of channels that can be used for periodic callbacks */

typedef struct {
    uint32_t period_us;
    timer_hf_cb_t callback;
} timer_callback_t;

typedef struct {
    timer_callback_t timer_callback[TIMER_HF_CB_CHANS];
    bool waiting;
} timer_vars_t;

//=========================== variables ========================================

static timer_vars_t _timer_vars;

//=========================== public ===========================================

/**
 * @brief Configure a high frequency timer with microsecond precision
 */
void db_timer_hf_init(void) {
    // No delay is running after initialization
    _timer_vars.waiting = false;

    // Configure and start High Frequency clock
    db_hfclk_init();

    // Configure the timer
    TIMER_HF->TASKS_CLEAR   = 1;
    TIMER_HF->PRESCALER     = 4;    // Run TIMER at 1MHz
    TIMER_HF->BITMODE       = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
    TIMER_HF->INTENSET      = (TIMER_INTENSET_COMPARE5_Enabled << TIMER_INTENSET_COMPARE5_Pos);
    NVIC_EnableIRQ(TIMER_HF_IRQ);

    // Start the timer
    TIMER_HF->TASKS_START = 1;
}

/**
 * @brief Set a callback to be called periodically using the high frequency timer
 *
 * @param[in] channel   TIMER channel used
 * @param[in] us        periodicity in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_periodic(uint8_t channel, uint32_t us, timer_hf_cb_t cb) {
    assert(channel >= 0 && channel < TIMER_HF_CB_CHANS);  // Make sure the required channel is correct
    assert(cb); // Make sure the callback function is valid

    _timer_vars.timer_callback[channel].period_us    = us;
    _timer_vars.timer_callback[channel].callback     = cb;
    TIMER_HF->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + channel));
    TIMER_HF->TASKS_CAPTURE[channel] = 1;
    TIMER_HF->CC[channel] += _timer_vars.timer_callback[channel].period_us;
}

/**
 * @brief Add a delay in us using the high frequency timer
 *
 * @param[in] us    delay in us
 */
void db_timer_hf_delay_us(uint32_t us) {
    TIMER_HF->TASKS_CAPTURE[5] = 1;
    TIMER_HF->CC[5] += us;
    _timer_vars.waiting = true;
    while (_timer_vars.waiting) {
        __WFE();
        __SEV();
        __WFE();
    }
}

/**
 * @brief Add a delay in milliseconds using the high frequency timer
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_hf_delay_ms(uint32_t ms) {
    db_timer_hf_delay_us(ms * 1000UL);
}

/**
 * @brief Add a delay in seconds using the high frequency timer
 *
 * @param[in] s delay in seconds
 */
void db_timer_hf_delay_s(uint32_t s) {
    db_timer_hf_delay_us(s * 1000UL * 1000UL);
}

//=========================== interrupt ========================================

void TIMER_HF_ISR(void) {
    if (TIMER_HF->EVENTS_COMPARE[5] == 1) {
        TIMER_HF->EVENTS_COMPARE[5] = 0;
        _timer_vars.waiting = false;
        __SEV();
    }

    for (uint8_t channel = 0; channel < TIMER_HF_CB_CHANS; ++channel) {
        if (TIMER_HF->EVENTS_COMPARE[channel] == 1) {
            TIMER_HF->EVENTS_COMPARE[channel] = 0;
            TIMER_HF->CC[channel] += _timer_vars.timer_callback[channel].period_us;
            if (_timer_vars.timer_callback[channel].callback) {
                _timer_vars.timer_callback[channel].callback();
            }
        }
    }
}
