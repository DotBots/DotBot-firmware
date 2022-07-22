#ifndef __PID_H
#define __PID_H

/**
 * @file pid.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "pid" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdbool.h>
#include <stdint.h>

//=========================== defines ==========================================

typedef enum {
    DB_PID_MODE_AUTO = 0,               ///< Automatic PID enabled
    DB_PID_MODE_MANUAL,                 ///< Manual mode
} pid_mode_t;

typedef enum {
    DB_PID_DIRECTION_DIRECT = 0,        ///< Direct direction
    DB_PID_DIRECTION_REVERSED,          ///< Reversed direction
} pid_direction_t;

typedef struct {
    float kp;                           ///< Value factor
    float ki;                           ///< Integral value factor
    float kd;                           ///< Derivative value factor
} pid_gains_t;

typedef struct {
    float output_sum;                   ///< Cumulative output values (used to keep history in integration component)
    float last_input;                   ///< Input used in previous iteration (used by the derivative component)
} pid_state_t;

typedef struct {
    pid_gains_t gains;                  ///< Factor parameters of the PID
    pid_mode_t mode;                    ///< Current mode, auto or manual
    pid_direction_t direction;          ///< Direction
    pid_state_t state;                  ///< Internal state
    uint32_t sample_time;               ///< Sampling time in milliseconds
    float input;                        ///< Current input value
    float output;                       ///< Last computed output
    float target;                       ///< Current target
    float output_min;                   ///< Minimal value allowed for output
    float output_max;                   ///< Maximal value allowed for output
} pid_t;

//=========================== prototypes =======================================

void db_pid_init(pid_t *pid, float input, float target,
                float kp, float ki, float kd,
                float output_min, float output_max,
                uint32_t sample_time,
                pid_mode_t mode, pid_direction_t direction);
void db_pid_update(pid_t *pid);
void db_pid_set_gains(pid_t *pid, const pid_gains_t *gains);
void db_pid_set_sample_time(pid_t *pid, uint32_t sample_time);
void db_pid_set_output_limits(pid_t *pid, float output_min, float output_max);
void db_pid_set_mode(pid_t *pid, pid_mode_t mode);
void db_pid_set_direction(pid_t *pid, pid_direction_t direction);

#endif
