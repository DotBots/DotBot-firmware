/**
 * @file motors.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "motors bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "motors.h"

//=========================== defines =========================================

// Motor driver pin definition
#define AIN1_PIN  2UL
#define AIN1_PORT 0UL
#define AIN2_PIN  28UL
#define AIN2_PORT 0UL
#define BIN1_PIN  9UL
#define BIN1_PORT 1UL
#define BIN2_PIN  11UL
#define BIN2_PORT 0UL

// Max value of the PWM counter register.
#define M_TOP 100

//=========================== variables =========================================

// Variable that stores the PWM duty cycle for all four PWM channels

static motors_vars_t motors_vars;

//=========================== public ==========================================

/**
 * @brief Configures the PMW0 peripheral to work with the onboard DotBot RGB Motor driver
 *
 * The DotBot uses a DRV8833 dual H-bridge driver with a 4 pmw control interface.
 * the PWM0 peripheral is used to generate the pwm signals it requires.
 *
 * PWM frequency = 10Khz
 * PWM resolution = 100 units (1us resolution)
 *
 */
void db_motors_init(void) {

    // Set the DotBot-specific motor pins
    motors_vars.pin[0]  = AIN1_PIN;
    motors_vars.port[0] = AIN1_PORT;
    motors_vars.pin[1]  = AIN2_PIN;
    motors_vars.port[1] = AIN2_PORT;
    motors_vars.pin[2]  = BIN1_PIN;
    motors_vars.port[2] = BIN1_PORT;
    motors_vars.pin[3]  = BIN2_PIN;
    motors_vars.port[3] = BIN2_PORT;
    // Internal M_TOP value of 100
    motors_vars.mtop = M_TOP;
    // Init PWM module with internal values
    db_pwm_init(&motors_vars);
}

void db_pwm_init(motors_vars_t *ctx) {

    // Configure the PWM pins as output in the GPIO peripheral.
    if (ctx->pin[0] != PIN_INVALID && ctx->port[0] != PIN_INVALID) {
        // Set corresponding GPIO as output
        NRF_P0->DIRSET = 1 << ctx->pin[0];

        // Output pin config
        NRF_PWM0->PSEL.OUT[0] = (ctx->port[0] << PWM_PSEL_OUT_PORT_Pos) |
                                (ctx->pin[0] << PWM_PSEL_OUT_PIN_Pos) |
                                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    }

    if (ctx->pin[1] != PIN_INVALID && ctx->port[1] != PIN_INVALID) {
        // Set corresponding GPIO as output
        NRF_P0->DIRSET = 1 << ctx->pin[1];

        // Output pin config
        NRF_PWM0->PSEL.OUT[1] = (ctx->port[1] << PWM_PSEL_OUT_PORT_Pos) |
                                (ctx->pin[1] << PWM_PSEL_OUT_PIN_Pos) |
                                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    }

    if (ctx->pin[2] != PIN_INVALID && ctx->port[2] != PIN_INVALID) {
        // Set corresponding GPIO as output
        NRF_P0->DIRSET = 1 << ctx->pin[2];

        // Output pin config
        NRF_PWM0->PSEL.OUT[2] = (ctx->port[2] << PWM_PSEL_OUT_PORT_Pos) |
                                (ctx->pin[2] << PWM_PSEL_OUT_PIN_Pos) |
                                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    }

    if (ctx->pin[3] != PIN_INVALID && ctx->port[3] != PIN_INVALID) {
        // Set corresponding GPIO as output
        NRF_P0->DIRSET = 1 << ctx->pin[3];

        // Output pin config
        NRF_PWM0->PSEL.OUT[3] = (ctx->port[3] << PWM_PSEL_OUT_PORT_Pos) |
                                (ctx->pin[3] << PWM_PSEL_OUT_PIN_Pos) |
                                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    }

    // Enable the PWM peripheral
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);

    // Configure PWM frequency and counting mode
    NRF_PWM0->PRESCALER  = PWM_PRESCALER_PRESCALER_DIV_16;               // 1MHz clock
    NRF_PWM0->COUNTERTOP = ctx->mtop;                                    // period for the PWM signal (10kHz)
    NRF_PWM0->MODE       = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);  // UP counting mode
    NRF_PWM0->LOOP       = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);  // Disable single sequence looping feature

    // Configure how many, and how the PWM dutycycles are loaded from memory
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |   // Have a different duty cycle value for each channel.
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);  // Reload the duty cycle values after every period, no delay

    // Configure the EasyDMA variables for loading the duty cycle values.
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(ctx->pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = ((sizeof(ctx->pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM0->SEQ[0].REFRESH  = 0UL;
    NRF_PWM0->SEQ[0].ENDDELAY = 0UL;

    // Activate the automatic looping of the PWM duty cycle sequence.
    NRF_PWM0->SHORTS = (PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos);

    // For safety, initialize all PWMs to zero.
    // Assigning values must go between 0 and M_TOP. the "| 1 <<15" is to set the polarity of the pwm waveform,
    ctx->pwm_seq[0] = 0 | 1 << 15;
    ctx->pwm_seq[1] = 0 | 1 << 15;
    ctx->pwm_seq[2] = 0 | 1 << 15;
    ctx->pwm_seq[3] = 0 | 1 << 15;
}

/**
 * @brief Set the percentage speed of the right and left motors on the DotBot
 *
 *  Each motor input variable receives a percentage speed from -100 to 100.
 *  Positive values turn the motor forward.
 *  Negative values turn the motor backward.
 *  Zero, stops the motor
 *
 * @param[in] l_speed speed of the left motor [-100, 100]
 * @param[in] r_speed speed of the left motor [-100, 100]
 */
void db_motors_set_speed(int16_t l_speed, int16_t r_speed) {

    // Double check for out-of-bound values.
    if (l_speed > 100)
        l_speed = 100;
    if (r_speed > 100)
        r_speed = 100;

    if (l_speed < -100)
        l_speed = -100;
    if (r_speed < -100)
        r_speed = -100;

    // Left motor processing
    if (l_speed >= 0)  // Positive values turn the motor forward.
    {
        motors_vars.pwm_seq[0] = l_speed | 1 << 15;
        motors_vars.pwm_seq[1] = 0 | 1 << 15;
    }
    if (l_speed < 0)  // Negative values turn the motor backward.
    {
        l_speed *= -1;  // remove the negative before loading into memory

        motors_vars.pwm_seq[0] = 0 | 1 << 15;
        motors_vars.pwm_seq[1] = l_speed | 1 << 15;
    }

    // Right motor processing
    if (r_speed >= 0)  // Positive values turn the motor forward.
    {
        motors_vars.pwm_seq[2] = r_speed | 1 << 15;
        motors_vars.pwm_seq[3] = 0 | 1 << 15;
    }
    if (r_speed < 0)  // Negative values turn the motor backward.
    {
        r_speed *= -1;  // remove the negative before loading into memory

        motors_vars.pwm_seq[2] = 0 | 1 << 15;
        motors_vars.pwm_seq[3] = r_speed | 1 << 15;
    }

    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}

/**
 * @brief Low-level set of the PWM pulse length on a specific channel.
 *
 *
 * @param[in] ctx Pointer to the motors_vars_t context.
 * @param[in] channel PWM channel [0,3].
 * @param[in] pwm_length PWM pulse length.
 */
void db_motors_set_pwm_length(motors_vars_t *ctx, uint8_t channel, uint16_t pwm_length) {
    if (channel > 3) {
        return;
    }

    if (ctx != NULL) {
        ctx->pwm_seq[channel] = pwm_length | 1 << 15;

        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
    }
}
