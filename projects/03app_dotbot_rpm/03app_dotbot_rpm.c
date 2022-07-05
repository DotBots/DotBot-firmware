/**
 * @file 03app_dotbot.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is the main DotBot app.
 *
 * Load this program on your board. Now the DotBot can be remote controlled 
 * from a nearby nRF52840-DK. THe buttons of the DK serving as Forward, 
 * Right, Left and Back buttons.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <nrf.h>
#include "motors.h"
#include "board.h"
#include "pid.h"
#include "rpm.h"
#include "timer_hf.h"

//=========================== defines ==========================================

#define PID_SAMPLE_TIME_MS          (50)

//=========================== variables ========================================

static pid_t _pid_right = { 0 };
static pid_t _pid_left = { 0 };
static const pid_parameters_t _pid_params = { .kp = 2, .ki = 5, .kd = 1 };
static rpm_values_t _rpm = { 0 };

//=========================== prototypes =======================================

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_timer_hf_init();

    // Turn ON the DotBot board regulator
    db_board_init();

    // Configure Motors
    db_motors_init();

    // Configure RPM driver
    db_rpm_init();

    // Initialize the pid
    db_pid_init(&_pid_right, _rpm.right.rps, 0.0, _pid_params.kp, _pid_params.ki, _pid_params.kd,
                PID_SAMPLE_TIME_MS, 0, 70, DB_PID_MODE_AUTO, DB_PID_DIRECTION_DIRECT);

    uint16_t loop_group = 10000;
    uint16_t loop_idx = 0;
    uint16_t target_idx = 0;

    // Wait for radio packets to arrive/
    while (1) {
        db_rpm_get_values(&_rpm);
        _pid_right.input = _rpm.right.speed;
        _pid_right.target = target_idx % 2 ? 60 : 40;
        loop_idx++;
        if (loop_idx % loop_group == 0) {
             target_idx++;
        }
        db_pid_update(&_pid_right);
        printf("Target - right: %u, left: %u\n", (int16_t)_pid_right.target, (int16_t)_pid_right.target);
        printf("RPS - right: %u, left: %u\n", (uint32_t)_rpm.right.speed, (uint32_t)_rpm.right.speed);
        printf("Speed - right: %i, left: %i\n", (int16_t)_pid_right.output, (int16_t)_pid_right.output);
        puts("");
        db_motors_set_speed((int16_t)_pid_right.output, (int16_t)_pid_right.output);
        db_timer_hf_delay_ms(PID_SAMPLE_TIME_MS);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions ========================================
