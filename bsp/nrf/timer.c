/**
 * @file
 * @ingroup bsp_timer
 *
 * @brief  nRF52833-specific definition of the "timer" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <nrf.h>
#include <nrf_peripherals.h>
#include "clock.h"
#include "timer.h"

//=========================== define ===========================================

#define TIMER_MAX_CHANNELS (4U)
#define TIMER_IRQ_PRIORITY (3U)

typedef struct {
    NRF_RTC_Type *p;
    IRQn_Type     irq;
    uint8_t       cc_num;
} timer_conf_t;

typedef struct {
    uint32_t   period_ticks;  ///< Period in ticks between each callback
    bool       one_shot;      ///< Whether this is a one shot callback
    timer_cb_t callback;      ///< Pointer to the callback function
} timer_callback_t;

typedef struct {
    timer_callback_t timer_callback[TIMER_MAX_CHANNELS];  ///< List of timer callback structs
    bool             running;                             ///< Whether the delay timer is running
} timer_vars_t;

//=========================== prototypes =======================================

static uint32_t _ms_to_ticks(uint32_t ms);

//=========================== variables ========================================

static const timer_conf_t _devs[RTC_COUNT] = {
#if defined(NRF5340_XXAA)
    {
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_RTC0_NS,
#else
        .p = NRF_RTC0_S,
#endif
        .irq    = RTC0_IRQn,
        .cc_num = RTC0_CC_NUM - 1,
    },
    {
#if defined(NRF_NETWORK) || defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_RTC1_NS,
#else
        .p = NRF_RTC1_S,
#endif
        .irq    = RTC1_IRQn,
        .cc_num = RTC1_CC_NUM - 1,
    },
#else
    {
        .p      = NRF_RTC0,
        .irq    = RTC0_IRQn,
        .cc_num = RTC0_CC_NUM - 1,
    },
    {
        .p      = NRF_RTC1,
        .irq    = RTC1_IRQn,
        .cc_num = RTC1_CC_NUM - 1,
    },
    {
        .p      = NRF_RTC2,
        .irq    = RTC2_IRQn,
        .cc_num = RTC2_CC_NUM - 1,
    },
#endif
};

static timer_vars_t _timer_vars[RTC_COUNT] = { 0 };

//=========================== public ===========================================

void db_timer_init(timer_t timer) {
    // No delay is running after initialization
    _timer_vars[timer].running = false;

#if !defined(USE_SWARMIT)
    // Configure and start Low Frequency clock
    db_lfclk_init();
#endif

    // Configure the RTC
    _devs[timer].p->TASKS_STOP  = 1;
    _devs[timer].p->TASKS_CLEAR = 1;
    _devs[timer].p->PRESCALER   = 0;  // Run RTC at 32768Hz
    _devs[timer].p->INTENSET    = (1 << (RTC_INTENSET_COMPARE0_Pos + _devs[timer].cc_num));
    NVIC_EnableIRQ(_devs[timer].irq);
    NVIC_SetPriority(_devs[timer].irq, TIMER_IRQ_PRIORITY);

    // Start the timer
    db_timer_start(timer);
}

void db_timer_start(timer_t timer) {
    _devs[timer].p->TASKS_START = 1;
}

void db_timer_stop(timer_t timer) {
    _devs[timer].p->TASKS_STOP = 1;
}

uint32_t db_timer_ticks(timer_t timer) {
    return _devs[timer].p->COUNTER;
}

void db_timer_set_periodic_ms(timer_t timer, uint8_t channel, uint32_t ms, timer_cb_t cb) {
    assert(channel >= 0 && channel < _devs[timer].cc_num);  // Make sure the required channel is correct
    assert(cb);                                             // Make sure the callback function is valid

    _timer_vars[timer].timer_callback[channel].period_ticks = _ms_to_ticks(ms);
    _timer_vars[timer].timer_callback[channel].one_shot     = false;
    _timer_vars[timer].timer_callback[channel].callback     = cb;
    _devs[timer].p->INTENSET                                = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    _devs[timer].p->CC[channel]                             = _devs[timer].p->COUNTER + _timer_vars[timer].timer_callback[channel].period_ticks;
}

void db_timer_set_oneshot_ticks(timer_t timer, uint8_t channel, uint32_t ticks, timer_cb_t cb) {
    assert(channel >= 0 && channel < _devs[timer].cc_num);  // Make sure the required channel is correct
    assert(cb);                                             // Make sure the callback function is valid

    _timer_vars[timer].timer_callback[channel].period_ticks = ticks;
    _timer_vars[timer].timer_callback[channel].one_shot     = true;
    _timer_vars[timer].timer_callback[channel].callback     = cb;
    _devs[timer].p->INTENSET                                = (1 << (RTC_INTENSET_COMPARE0_Pos + channel));
    _devs[timer].p->CC[channel]                             = _devs[timer].p->COUNTER + _timer_vars[timer].timer_callback[channel].period_ticks;
}

void db_timer_set_oneshot_ms(timer_t timer, uint8_t channel, uint32_t ms, timer_cb_t cb) {
    db_timer_set_oneshot_ticks(timer, channel, _ms_to_ticks(ms), cb);
}

void db_timer_set_oneshot_s(timer_t timer, uint8_t channel, uint32_t s, timer_cb_t cb) {
    db_timer_set_oneshot_ticks(timer, channel, s * 32768, cb);
}

void db_timer_delay_ticks(timer_t timer, uint32_t ticks) {
    _devs[timer].p->CC[_devs[timer].cc_num] = _devs[timer].p->COUNTER + ticks;
    _timer_vars[timer].running              = true;
    while (_timer_vars[timer].running) {
        // Let's go to sleep
        // See https://devzone.nordicsemi.com/f/nordic-q-a/49010/methods-to-put-the-nrf52-to-sleep-in-a-spinlock-loop
        // for details
        __WFE();
        __SEV();
        __WFE();
    }
}

void db_timer_delay_ms(timer_t timer, uint32_t ms) {
    db_timer_delay_ticks(timer, _ms_to_ticks(ms));
}

void db_timer_delay_s(timer_t timer, uint32_t s) {
    db_timer_delay_ticks(timer, s * 32768);
}

//=========================== private ==========================================

static uint32_t _ms_to_ticks(uint32_t ms) {
    // Return an estimation of the number of ticks matching the
    // required number of milliseconds. The estimation should be just above to
    // avoid firing the timer in advance.
    return (ms * 33) - (uint32_t)(ms * (float)(33 - (32768.0 / 1000)));
}

//=========================== interrupt ========================================

static void _timer_isr(timer_t timer) {
    if (_devs[timer].p->EVENTS_COMPARE[_devs[timer].cc_num] == 1) {
        _devs[timer].p->EVENTS_COMPARE[_devs[timer].cc_num] = 0;
        _timer_vars[timer].running                          = false;
        __SEV();
    }

    for (uint8_t channel = 0; channel < _devs[timer].cc_num; ++channel) {
        if (_devs[timer].p->EVENTS_COMPARE[channel] == 1) {
            _devs[timer].p->EVENTS_COMPARE[channel] = 0;
            if (_timer_vars[timer].timer_callback[channel].one_shot) {
                _devs[timer].p->INTENCLR = (1 << (RTC_INTENCLR_COMPARE0_Pos + channel));
            } else {
                _devs[timer].p->CC[channel] += _timer_vars[timer].timer_callback[channel].period_ticks;
            }
            if (_timer_vars[timer].timer_callback[channel].callback) {
                _timer_vars[timer].timer_callback[channel].callback();
            }
        }
    }
}

void RTC0_IRQHandler(void) {
    _timer_isr(0);
}

void RTC1_IRQHandler(void) {
    _timer_isr(1);
}

#if !defined(NRF5340_XXAA)
void RTC2_IRQHandler(void) {
    _timer_isr(2);
}
#endif
