/**
 * @file 00std_motors.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the motor driver in the DotBot board.
 *
 * Load this program on your board. The right wheel should spin forwards and then backward.
 * Afterwards the left wheel also turns forward and then backward.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== defines =========================================

// Pin definitions
// motors driver pins
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

// Define a blocking wait function.
#define WAIT_A_BIT(PAUSE)                    \
    for (int i = 0; i < 3000 * PAUSE; i++) { \
        ;                                    \
    }  ///< The 3000 magic number, approximates to about 1ms per 1 unit of PAUSE.

//=========================== variables =========================================

// Variable that stores the PWM duty cycle for all four PWM channels
uint16_t pwm_seq[4];

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Turn ON the DotBot board regulator
    NRF_P0->DIRSET = 1 << 20;
    NRF_P0->OUTSET = 1 << 20;

    // Configure the PWM pins as output in the GPIO peripheral.
    NRF_P0->DIRSET = 1 << AIN1_PIN;
    NRF_P0->DIRSET = 1 << AIN2_PIN;
    NRF_P0->DIRSET = 1 << BIN2_PIN;
    NRF_P0->DIRSET = 1 << BIN1_PIN;

    // Output pin config AIN1
    NRF_PWM0->PSEL.OUT[0] = (AIN1_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (AIN1_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Output pin config AIN2
    NRF_PWM0->PSEL.OUT[1] = (AIN2_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (AIN2_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Output pin config BIN1
    NRF_PWM0->PSEL.OUT[2] = (BIN1_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (BIN1_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // Output pin config BIN2
    NRF_PWM0->PSEL.OUT[3] = (BIN2_PORT << PWM_PSEL_OUT_PORT_Pos) |
                            (BIN2_PIN << PWM_PSEL_OUT_PIN_Pos) |
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

    while (1) {
        // Right motor forward
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 60 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(2000);  // wait 2 sec

        // Turnoff motor for a bit
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(1000);  // wait 1 sec

        // Right motor backward
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 60 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(2000);  // wait 2 sec

        // Turnoff motor for a bit
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(1000);  // wait 1 sec

        // Left motor forward
        pwm_seq[0] = 60 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(2000);  // wait 2 sec

        // Turnoff motor for a bit
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(1000);  // wait 1 sec

        // Left motor backward
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 60 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(2000);  // wait 2 sec

        // Turnoff motor for a bit
        pwm_seq[0] = 0 | 1 << 15;
        pwm_seq[1] = 0 | 1 << 15;
        pwm_seq[2] = 0 | 1 << 15;
        pwm_seq[3] = 0 | 1 << 15;
        // Update PWM values
        NRF_PWM0->TASKS_SEQSTART[0] = PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger;
        WAIT_A_BIT(1000);  // wait 1 sec
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
