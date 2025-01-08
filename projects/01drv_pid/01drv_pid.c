/**
 * @file
 * @ingroup samples_drv
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
#include <stdint.h>
#include <nrf.h>
#include "motors.h"
#include "board.h"
#include "pid.h"
#include "rpm.h"
#include "timer_hf.h"

//=========================== defines ==========================================

#define PID_SAMPLE_TIME_MS   (100)
#define PID_SPEED_TARGET     (60)
#define MOTORS_SPEED_INITIAL (45)

//=========================== variables ========================================

static pid_t             _pid_left   = { 0 };
static pid_t             _pid_right  = { 0 };
static rpm_values_t      _rpm        = { 0 };
static const pid_gains_t _pid_params = {
    .kp = 1.5,
    .ki = 2,
    .kd = 0,
};

//=========================== main =============================================

int main(void) {
    // Initialize high frequency timer used to loop sample time delays
    db_timer_hf_init(0);

    // Turn ON the DotBot board regulator
    db_board_init();

    // Configure Motors
    db_motors_init();

    // Configure RPM driver
    db_rpm_init();

    // Initialize the pids
    db_pid_init(&_pid_left, 0.0, PID_SPEED_TARGET,
                _pid_params.kp, _pid_params.ki, _pid_params.kd,
                0.0, 100.0, PID_SAMPLE_TIME_MS, DB_PID_MODE_AUTO, DB_PID_DIRECTION_DIRECT);
    db_pid_init(&_pid_right, 0.0, PID_SPEED_TARGET,
                _pid_params.kp, _pid_params.ki, _pid_params.kd,
                0.0, 100.0, PID_SAMPLE_TIME_MS, DB_PID_MODE_AUTO, DB_PID_DIRECTION_DIRECT);

    // PID update loop
    while (1) {
        db_rpm_get_values(&_rpm);
        _pid_right.input = _rpm.right.speed;
        _pid_left.input  = _rpm.left.speed;

        db_pid_update(&_pid_left);
        db_pid_update(&_pid_right);
        printf("Speed left RPM    : %f\n", _rpm.left.speed);
        printf("Speed left motors : %i\n", (int16_t)_pid_left.output);
        printf("Speed right RPM   : %f\n", _rpm.right.speed);
        printf("Speed right motors: %i\n", (int16_t)_pid_right.output);
        puts("");

        db_motors_set_speed((int16_t)_pid_left.output, (int16_t)_pid_right.output);
        db_timer_hf_delay_ms(0, PID_SAMPLE_TIME_MS);
    }
}
