/**
 * @file pwm.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "pwm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <nrf.h>

#include "gpio.h"
#include "pwm.h"

//=========================== defines ==========================================

#define DB_PWM              (NRF_PWM0)
#define DB_PWM_MAX_CHANNELS (4)

// Variable that stores the PWM duty cycle for all four PWM channels

typedef struct {
    uint16_t seq[4];
} pwm_vars_t;

//=========================== variables ========================================

static pwm_vars_t pwm_vars;

//=========================== public ===========================================

void db_pwm_init(const gpio_t *pins, size_t num_channels, uint16_t mtop) {

    // configure PWM channel pins;
    for (uint8_t channel = 0; channel < num_channels; channel++) {
        // Configure the PWM pins as output.
        db_gpio_init(&pins[channel], DB_GPIO_OUT);
        NRF_PWM0->PSEL.OUT[channel] = (pins[channel].port << PWM_PSEL_OUT_PORT_Pos) |
                                      (pins[channel].pin << PWM_PSEL_OUT_PIN_Pos) |
                                      (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    }

    // Enable the PWM peripheral
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);

    // Configure PWM frequency and counting mode
    NRF_PWM0->PRESCALER  = PWM_PRESCALER_PRESCALER_DIV_16;               // 1MHz clock
    NRF_PWM0->COUNTERTOP = mtop;                                         // for example 100us period for the PWM signal means 10kHz
    NRF_PWM0->MODE       = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);  // UP counting mode
    NRF_PWM0->LOOP       = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);  // Disable single sequence looping feature

    // Configure how many, and how the PWM dutycycles are loaded from memory
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |   // Have a different duty cycle value for each channel.
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);  // Reload the duty cycle values after every period, no delay

    // Configure the EasyDMA variables for loading the duty cycle values.
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_vars.seq) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = (DB_PWM_MAX_CHANNELS << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM0->SEQ[0].REFRESH  = 0UL;
    NRF_PWM0->SEQ[0].ENDDELAY = 0UL;

    // Activate the automatic looping of the PWM duty cycle sequence.
    NRF_PWM0->SHORTS = (PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos);

    // For safety, initialize all PWMs to zero.
    for (uint8_t channel = 0; channel < DB_PWM_MAX_CHANNELS; channel++) {
        // Assigning values must go between 0 and M_TOP (100). the "| 1 <<15" is to set the polarity of the pwm waveform,
        // This way a value of 30 means the waveform is High 30% of the time, and the idle value of the PWM channels is 0v.
        pwm_vars.seq[channel] = 0 | 1 << 15;
    }
}

void db_pwm_channel_set(uint8_t channel, uint16_t value) {
    assert(channel < DB_PWM_MAX_CHANNELS);

    pwm_vars.seq[channel] = value | 1 << 15;
    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}

void db_pwm_channels_set(uint16_t *values) {
    assert(sizeof(values) == DB_PWM_MAX_CHANNELS);

    for (uint8_t channel = 0; channel < DB_PWM_MAX_CHANNELS; channel++) {
        pwm_vars.seq[channel] = values[channel] | 1 << 15;
    }
    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}
