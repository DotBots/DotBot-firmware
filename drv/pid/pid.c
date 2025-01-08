/**
 * @file
 * @ingroup drv_pid
 *
 * @brief  nRF52833-specific definition of the "pid" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <assert.h>
#include <stdint.h>
#include "pid.h"
#include <nrf.h>

void db_pid_init(pid_t *pid, float input, float target,
                 float kp, float ki, float kd,
                 float output_min, float output_max,
                 uint32_t   sample_time,
                 pid_mode_t mode, pid_direction_t direction) {
    assert(target >= output_min);
    assert(target <= output_max);

    pid->input            = input;
    pid->target           = target;
    pid->state.last_input = input;
    pid->state.output_sum = 0;
    pid->mode             = mode;
    pid->sample_time      = sample_time;
    db_pid_set_output_limits(pid, output_min, output_max);
    pid_gains_t gains = { .kp = kp, .ki = ki, .kd = kd };
    db_pid_set_gains(pid, &gains);
    db_pid_set_direction(pid, direction);
}

void db_pid_update(pid_t *pid) {
    if (pid->mode == DB_PID_MODE_MANUAL) {
        return;
    }

    float error = pid->target - pid->input;
    pid->state.output_sum += pid->gains.ki * error;
    if (pid->state.output_sum > pid->output_max) {
        pid->state.output_sum = pid->output_max;
    } else if (pid->state.output_sum < pid->output_min) {
        pid->state.output_sum = pid->output_min;
    }
    float d_input = pid->input - pid->state.last_input;

    // PID output computation
    float result = pid->gains.kp * error + pid->state.output_sum + pid->gains.kd * d_input;
    if (result > pid->output_max) {
        result = pid->output_max;
    } else if (result < pid->output_min) {
        result = pid->output_min;
    }

    pid->state.last_input = pid->input;
    pid->output           = result;
}

void db_pid_set_gains(pid_t *pid, const pid_gains_t *gains) {
    if (gains->kp < 0 || gains->ki < 0 || gains->kd < 0) {
        return;
    }

    float sample_time_s = (float)pid->sample_time / 1000;
    pid->gains.kp       = gains->kp;
    pid->gains.ki       = gains->ki * sample_time_s;
    pid->gains.kd       = gains->kd / sample_time_s;

    if (pid->direction == DB_PID_DIRECTION_REVERSED) {
        pid->gains.kp = (0 - pid->gains.kp);
        pid->gains.ki = (0 - pid->gains.ki);
        pid->gains.kd = (0 - pid->gains.kd);
    }
}

void db_pid_set_sample_time(pid_t *pid, uint32_t sample_time) {
    assert(sample_time > 0);

    float time_ratio = (float)sample_time / pid->sample_time;
    pid->gains.ki *= time_ratio;
    pid->gains.kd /= time_ratio;
    pid->sample_time = sample_time;
}

void db_pid_set_output_limits(pid_t *pid, float output_min, float output_max) {
    assert(output_min < output_max);
    pid->output_min = output_min;
    pid->output_max = output_max;

    if (pid->mode == DB_PID_MODE_AUTO) {
        if (pid->output > output_max) {
            pid->output = output_max;
        } else if (pid->output < output_min) {
            pid->output = output_min;
        }
        if (pid->state.output_sum > output_max) {
            pid->state.output_sum = output_max;
        } else if (pid->state.output_sum < output_min) {
            pid->state.output_sum = output_min;
        }
    }
}

void db_pid_set_mode(pid_t *pid, pid_mode_t mode) {
    if (mode == pid->mode) {
        return;
    }

    pid->mode = mode;
    if (mode == DB_PID_MODE_AUTO) {
        db_pid_init(pid, pid->input, pid->target,
                    pid->gains.kp, pid->gains.ki, pid->gains.kd,
                    pid->output_min, pid->output_max,
                    pid->sample_time, pid->mode, pid->direction);
    }
}

void db_pid_set_direction(pid_t *pid, pid_direction_t direction) {
    if (direction == pid->direction) {
        return;
    }

    if (pid->mode == DB_PID_MODE_AUTO && pid->direction != direction) {
        pid->gains.kp = (0 - pid->gains.kp);
        pid->gains.ki = (0 - pid->gains.ki);
        pid->gains.kd = (0 - pid->gains.kd);
    }
    pid->direction = direction;
}
