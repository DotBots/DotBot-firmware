/**
 * @file rpm.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "rpm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include "rpm.h"

//=========================== defines =========================================

#ifndef RPM_LEFT_PIN
#define RPM_LEFT_PIN            (17)                /**< Number of the pin connected to the left magnetic encoder */
#endif

#ifndef RPM_LEFT_PORT
#define RPM_LEFT_PORT           (NRF_P0)            /**< Port of the pin connected to the left magnetic encoder */
#endif

#ifndef RPM_LEFT_TIMER
#define RPM_LEFT_TIMER          (NRF_TIMER0)        /**< Timer peripheral used to count left cycles */
#endif

#ifndef RPM_LEFT_PPI_CHAN
#define RPM_LEFT_PPI_CHAN       (0)                 /**< PPI channel used between left side timer and gpio */
#endif

#ifndef RPM_LEFT_GPIOTE_CHAN
#define RPM_LEFT_GPIOTE_CHAN    (0)                 /**< GPIOTE channel used for left side gpio event */
#endif

#ifndef RPM_RIGHT_PIN
#define RPM_RIGHT_PIN           (15)                /**< Number of the pin connected to the right magnetic encoder */
#endif

#ifndef RPM_RIGHT_PORT
#define RPM_RIGHT_PORT          (NRF_P0)            /**< Port of the pin connected to the right magnetic encoder */
#endif

#ifndef RPM_RIGHT_TIMER
#define RPM_RIGHT_TIMER         (NRF_TIMER1)        /**< Timer peripheral used to count right cycles */
#endif

#ifndef RPM_RIGHT_PPI_CHAN
#define RPM_RIGHT_PPI_CHAN      (1)                 /**< PPI channel used between right side timer and gpio */
#endif

#ifndef RPM_RIGHT_GPIOTE_CHAN
#define RPM_RIGHT_GPIOTE_CHAN   (1)                 /**< GPIOTE channel used for right side gpio event */
#endif

#ifndef RPM_RTC
#define RPM_RTC                 (NRF_RTC0)          /**< RTC peripheral used to sample cycle counts */
#endif

#ifndef RPM_RTC_IRQ
#define RPM_RTC_IRQ             (RTC0_IRQn)         /**< IRQ corresponding to the RTC used */
#endif

#ifndef RPM_RTC_ISR
#define RPM_RTC_ISR             (RTC0_IRQHandler)   /**< ISR function handler corresponding to the RTC used */
#endif

/**
 * Helper macro to compute speed in cm/s
 *
 * computed from the number of cycles measured within the last 125ms (one rotation is 3.77mm distance of the wheel)
 */
#define RPM_CYCLES_TO_SPEED(cycles)     (uint32_t)(37.7 * cycles / 8)

/**
 * Helper macro to compute rotation per minute
 *
 * 1 cycle corresponds to one rotation, so convert to the number of minutes, given the RTC frequency of 125ms
 */
#define RPM_CYCLES_TO_RPM(cycles)       (60 * 8 * cycles)

/**
 * Helper macro to compute rotation per second
 *
 * 1 cycle corresponds to one rotation, so convert to the number of seconds, given the RTC frequency of 125ms
 */
#define RPM_CYCLES_TO_RPS(cycles)       (8 * cycles)

/**
 * Helper struct used to store internal state variables
 */
typedef struct {
    uint32_t last_left_counts;
    uint32_t previous_left_counts;
    uint32_t last_right_counts;
    uint32_t previous_right_counts;
} rpm_vars_t;

//=========================== variables =========================================

/*
 * Global variable used to store cycle counts at the beginning and at the end of
 * an RTC timeframe (125ms), for each side
 */
static rpm_vars_t _rpm_vars;

//=========================== private ==========================================

/**
 * Compute the number of the left encoder cycles during the last 125ms time
 * frame. The function takes into account timer overflows.
 */
static inline uint32_t _db_rpm_left_cycles(void) {
    if (_rpm_vars.last_left_counts < _rpm_vars.previous_left_counts) {
        return UINT32_MAX - _rpm_vars.previous_left_counts + _rpm_vars.last_left_counts;
    }
    return _rpm_vars.last_left_counts - _rpm_vars.previous_left_counts;
}

/**
 * Compute the number of the right encoder cycles during the last 125ms time
 * frame. The function takes into account timer overflows.
 */
static inline uint32_t _db_rpm_right_cycles(void) {
    if (_rpm_vars.last_right_counts < _rpm_vars.previous_right_counts) {
        return UINT32_MAX - _rpm_vars.previous_right_counts + _rpm_vars.last_right_counts;
    }
    return _rpm_vars.last_right_counts - _rpm_vars.previous_right_counts;
}


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
void db_rpm_init(void) {
    // Configure pin connected to left magnetic encoder sensor, input pullup
    RPM_LEFT_PORT->PIN_CNF[RPM_LEFT_PIN]        |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    RPM_LEFT_PORT->PIN_CNF[RPM_LEFT_PIN]        &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[RPM_LEFT_GPIOTE_CHAN]     = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                                    (RPM_LEFT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                                    (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    // Configure pin connected to right magnetic encoder sensor, input pullup
    RPM_RIGHT_PORT->PIN_CNF[RPM_RIGHT_PIN]      |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    RPM_RIGHT_PORT->PIN_CNF[RPM_RIGHT_PIN]      &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[RPM_RIGHT_GPIOTE_CHAN]    = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                                    (RPM_RIGHT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                                    (GPIOTE_CONFIG_POLARITY_LoToHi  << GPIOTE_CONFIG_POLARITY_Pos);

    // Configure and clear timers
    // Timers are configured in counter mode with 32bit length
    RPM_LEFT_TIMER->MODE            = TIMER_MODE_MODE_LowPowerCounter;
    RPM_LEFT_TIMER->BITMODE         = TIMER_BITMODE_BITMODE_32Bit;
    RPM_LEFT_TIMER->INTENCLR        = 1;
    RPM_LEFT_TIMER->TASKS_CLEAR     = 1;

    RPM_RIGHT_TIMER->MODE           = TIMER_MODE_MODE_LowPowerCounter;
    RPM_RIGHT_TIMER->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;
    RPM_RIGHT_TIMER->INTENCLR       = 1;
    RPM_RIGHT_TIMER->TASKS_CLEAR    = 1;

    // Configure gpiote/timer count PPI
    //   - PO.17 (left magnetic encoder) event connected to PPI channel 0 which fires Timer 0 capture 0
    //   - PO.15 (right magnetic encoder) event connected to PPI channel 1 which fires Timer 0 capture 1
    NRF_PPI->CH[RPM_LEFT_PPI_CHAN].EEP  = (uint32_t)&NRF_GPIOTE->EVENTS_IN[RPM_LEFT_GPIOTE_CHAN];
    NRF_PPI->CH[RPM_LEFT_PPI_CHAN].TEP  = (uint32_t)&RPM_LEFT_TIMER->TASKS_COUNT;
    NRF_PPI->CH[RPM_RIGHT_PPI_CHAN].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[RPM_RIGHT_GPIOTE_CHAN];
    NRF_PPI->CH[RPM_RIGHT_PPI_CHAN].TEP = (uint32_t)&RPM_RIGHT_TIMER->TASKS_COUNT;

    // Enable PPI channels
    NRF_PPI->CHENSET = (1 << RPM_RIGHT_PPI_CHAN) | (1 << RPM_LEFT_PPI_CHAN);

    // Configure RTC with 125ms fire event delay
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;

    RPM_RTC->TASKS_STOP             = 1;
    RPM_RTC->TASKS_CLEAR            = 1;
    RPM_RTC->PRESCALER              = (1 << 12) - 1;
    RPM_RTC->CC[0]                  = 1;
    RPM_RTC->EVTENSET               = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    RPM_RTC->INTENSET               = (RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(RPM_RTC_IRQ);
    // Start RTC
    RPM_RTC->TASKS_START = 1;

    // Start timers used as counters
    RPM_LEFT_TIMER->TASKS_START = 1;
    RPM_RIGHT_TIMER->TASKS_START = 1;
}

static void update_counters(void) {
    _rpm_vars.previous_left_counts = _rpm_vars.last_left_counts;
    _rpm_vars.previous_right_counts = _rpm_vars.last_right_counts;
    RPM_LEFT_TIMER->TASKS_CAPTURE[0] = 1;
    RPM_RIGHT_TIMER->TASKS_CAPTURE[0] = 1;
    _rpm_vars.last_left_counts = RPM_LEFT_TIMER->CC[0];
    _rpm_vars.last_right_counts = RPM_RIGHT_TIMER->CC[0];
}

void RPM_RTC_ISR(void) {
    if (RPM_RTC->EVENTS_COMPARE[0] == 1) {
        RPM_RTC->EVENTS_COMPARE[0] = 0;
        update_counters();
        RPM_RTC->TASKS_CLEAR = 1;
    }
}

void db_rpm_get_values(rpm_values_t *values) {
    uint32_t left       = _db_rpm_left_cycles();
    uint32_t right      = _db_rpm_right_cycles();

    values->left.rpm    = RPM_CYCLES_TO_RPM(left);
    values->left.rps    = RPM_CYCLES_TO_RPS(left);
    values->left.speed  = RPM_CYCLES_TO_SPEED(left);

    values->right.rpm   = RPM_CYCLES_TO_RPM(right);
    values->right.rps   = RPM_CYCLES_TO_RPS(right);
    values->right.speed = RPM_CYCLES_TO_SPEED(right);
}
