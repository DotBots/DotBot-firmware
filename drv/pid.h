#ifndef __PID_H
#define __PID_H

/**
 * @defgroup    drv_pid    Proportional–integral–derivative (PID) controller
 * @ingroup     drv
 * @brief       Implementation of a PID controller
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>

//=========================== defines ==========================================

/// PID mode
typedef enum {
    DB_PID_MODE_AUTO = 0,  ///< Automatic PID enabled
    DB_PID_MODE_MANUAL,    ///< Manual mode
} pid_mode_t;

/// PID direction
typedef enum {
    DB_PID_DIRECTION_DIRECT = 0,  ///< Direct direction
    DB_PID_DIRECTION_REVERSED,    ///< Reversed direction
} pid_direction_t;

/// PID gains
typedef struct {
    float kp;  ///< Value factor
    float ki;  ///< Integral value factor
    float kd;  ///< Derivative value factor
} pid_gains_t;

/// PID internal state
typedef struct {
    float output_sum;  ///< Cumulative output values (used to keep history in integration component)
    float last_input;  ///< Input used in previous iteration (used by the derivative component)
} pid_state_t;

/// PID instance
typedef struct {
    pid_gains_t     gains;        ///< Factor parameters of the PID
    pid_mode_t      mode;         ///< Current mode, auto or manual
    pid_direction_t direction;    ///< Direction
    pid_state_t     state;        ///< Internal state
    uint32_t        sample_time;  ///< Sampling time in milliseconds
    float           input;        ///< Current input value
    float           output;       ///< Last computed output
    float           target;       ///< Current target
    float           output_min;   ///< Minimal value allowed for output
    float           output_max;   ///< Maximal value allowed for output
} pid_t;

//=========================== prototypes =======================================

/**
 * @brief   Initialize a PID control loop
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] input         Initial input value
 * @param[in] target        Target value
 * @param[in] kp            Gain applied to the input value
 * @param[in] ki            Gain applied to the integrated term
 * @param[in] kd            Gain applied to the derivative term
 * @param[in] output_min    Minimum output value
 * @param[in] output_max    Maximum output value
 * @param[in] sample_time   Sampling time of the pid (correspond to the update rate)
 * @param[in] mode          PID mode (manual or automatic)
 * @param[in] direction     Direction of the PID (direct or reversed)
 */
void db_pid_init(pid_t *pid, float input, float target,
                 float kp, float ki, float kd,
                 float output_min, float output_max,
                 uint32_t   sample_time,
                 pid_mode_t mode, pid_direction_t direction);

/**
 * @brief   Update the PID state
 *
 * Input attribute of the PID should be updated before calling this function
 *
 * @param[inout]    pid         Pointer to the pid struct
 */
void db_pid_update(pid_t *pid);

/**
 * @brief   Set PID gains
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] gains         Pointer to the gains struct
 */
void db_pid_set_gains(pid_t *pid, const pid_gains_t *gains);

/**
 * @brief   Set PID sample time (in milliseconds)
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] sample_time   Sample time in milliseconds (must be > 0)
 */
void db_pid_set_sample_time(pid_t *pid, uint32_t sample_time);

/**
 * @brief   Set PID output limits
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] output_min    Minimum value of output
 * @param[in] output_max    Maximum value of output
 */
void db_pid_set_output_limits(pid_t *pid, float output_min, float output_max);

/**
 * @brief   Update the mode of the PID (manual or automatic)
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] mode          New mode
 */
void db_pid_set_mode(pid_t *pid, pid_mode_t mode);

/**
 * @brief   Update the direction of the PID
 *
 * @param[in] pid           Pointer to the pid struct
 * @param[in] direction     New direction
 */
void db_pid_set_direction(pid_t *pid, pid_direction_t direction);

#endif
