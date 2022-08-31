/**
 * @file 00std_motors.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the PWM driver on nRF52840.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include "servos.h"

//=========================== defines ===================================

// Pin definitions
// motors driver pins
#define RUDDER_PORT 0UL
#define RUDDER_PIN  29UL

#define SAIL_PORT 0UL
#define SAIL_PIN  28UL

#define RUDDER_POSITION_UPPER_BOUND 1840
#define RUDDER_POSITION_LOWER_BOUND 840

#define SAIL_POSITION_UPPER_BOUND 2100
#define SAIL_POSITION_LOWER_BOUND 1200

// Max value of the PWM counter register.
#define M_TOP 20000

enum servo_position {
    SERVO_RUDDER = 0,
    SERVO_SAILS  = 1,
};

//=========================== prototypes ==================================

void turn_rudder(uint16_t pwm_length);
void turn_sail(uint16_t pwm_length);

//========================== variables ====================================

// Variable that stores the PWM duty cycle for all four PWM channels
uint16_t pwm_seq[4];

/**
 *  @brief Initialization routine of the PWM module.
 */
void servos_init(void) {
    // Configure the PWM pins as output in the GPIO peripheral.
    NRF_P0->DIRSET = 1 << RUDDER_PIN;
    NRF_P0->OUTSET = 1 << SAIL_PIN;

    // Output pin config AIN1
    NRF_PWM0->PSEL.OUT[0] = (RUDDER_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (RUDDER_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Output pin config AIN1
    NRF_PWM0->PSEL.OUT[1] = (SAIL_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (SAIL_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Enable the PWM peripheral
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);

    // Configure PWM frequency and counting mode
    NRF_PWM0->PRESCALER  = PWM_PRESCALER_PRESCALER_DIV_16;               // 1MHz clock
    NRF_PWM0->COUNTERTOP = M_TOP;                                        // 100us period for the PWM signal (10kHz)
    NRF_PWM0->MODE       = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);  // UP counting mode
    NRF_PWM0->LOOP       = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);

    // Configure how many, and how the PWM dutycycles are loaded from memory
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |   // Have a different duty cycle value for each channel.
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);  // Reload the duty cycle values after every period, no delay

    // Configure the EasyDMA variables for loading the duty cycle values.
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    NRF_PWM0->SEQ[0].REFRESH  = 0UL;
    NRF_PWM0->SEQ[0].ENDDELAY = 0UL;

    // Activate the automatic looping of the PWM duty cycle sequence.
    NRF_PWM0->SHORTS = (PWM_SHORTS_LOOPSDONE_SEQSTART0_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART0_Pos);

    // For safety, initialize all PWMs to zero.
    // Assigning values must go between 0 and M_TOP (100). the "| 1 <<15" is to set the polarity of the pwm waveform,
    // This way a value of 30 means 30% high, and the idle value of the PWM channels is 0.
    pwm_seq[0] = 0 | 1 << 15;
    pwm_seq[1] = 0 | 1 << 15;
    pwm_seq[2] = 0 | 1 << 15;
    pwm_seq[3] = 0 | 1 << 15;

    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}

// private function
void turn_rudder(uint16_t pwm_length) {
    pwm_seq[SERVO_RUDDER] = pwm_length | 1 << 15;

    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}

// private function
void turn_sail(uint16_t pwm_length) {
    pwm_seq[SERVO_SAILS] = pwm_length | 1 << 15;

    // Update PWM values
    NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
}

void servos_rudder_turn(int8_t angle) {
    uint16_t pwm_length;

    pwm_length = (angle * (RUDDER_POSITION_UPPER_BOUND - RUDDER_POSITION_LOWER_BOUND)) / 255;
    pwm_length += (RUDDER_POSITION_UPPER_BOUND + RUDDER_POSITION_LOWER_BOUND) / 2;

    // make sure imprecision in calculation does not cause a position out of bounds
    if (pwm_length > RUDDER_POSITION_UPPER_BOUND) {
        pwm_length = RUDDER_POSITION_UPPER_BOUND;
    }
    if (pwm_length < RUDDER_POSITION_LOWER_BOUND) {
        pwm_length = RUDDER_POSITION_LOWER_BOUND;
    }

    turn_rudder(pwm_length);
}

void servos_sail_trim(int8_t angle) {
    uint16_t pwm_length;

    pwm_length = (angle * (SAIL_POSITION_UPPER_BOUND - SAIL_POSITION_LOWER_BOUND)) / 255;
    pwm_length += (SAIL_POSITION_UPPER_BOUND + SAIL_POSITION_LOWER_BOUND) / 2;

    // make sure imprecision in calculation does not cause a position out of bounds
    if (pwm_length > SAIL_POSITION_UPPER_BOUND) {
        pwm_length = SAIL_POSITION_UPPER_BOUND;
    }
    if (pwm_length < SAIL_POSITION_LOWER_BOUND) {
        pwm_length = SAIL_POSITION_LOWER_BOUND;
    }

    turn_sail(pwm_length);
}
