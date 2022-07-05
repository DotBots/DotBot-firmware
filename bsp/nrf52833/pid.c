/**
 * @file pid.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "pid" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <nrf.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

void db_pid_init(pid_t *pid, float input, float target,
                    float kp, float ki, float kd,
                    uint32_t sample_time, float output_min, float output_max,
                    pid_mode_t mode, pid_direction_t direction) {
    assert(target >= output_min);
    assert(target <= output_max);

    pid->input = input;
    pid->target = target;
    pid->sample_time = sample_time;
    pid->state.last_input = input;
    pid->mode = mode;
    db_pid_set_output_limits(pid, output_min, output_max);
    pid_parameters_t parameters = {.kp = kp, .ki = ki, .kd = kd};
    db_pid_set_parameters(pid, &parameters);
    db_pid_set_direction(pid, direction);
}

void db_pid_update(pid_t *pid) {
    if (pid->mode == DB_PID_MODE_MANUAL) {
        return;
    }

    float error = pid->target - pid->input;
    pid->state.output_sum += pid->parameters.ki * error;
    if (pid->state.output_sum > pid->output_max) {
        pid->state.output_sum = pid->output_max;
    }
    else if (pid->state.output_sum < pid->output_min) {
        pid->state.output_sum = pid->output_min;
    }

    float d_input = pid->input - pid->state.last_input;

    // PID output computation
    float result = pid->parameters.kp * error + pid->state.output_sum - pid->parameters.kd * d_input;
    if (result > pid->output_max) {
        result = pid->output_max;
    }
    else if (result < pid->output_min) {
        result = pid->output_min;
    }

    pid->state.last_input = pid->input;
    pid->output = result;
}

void db_pid_set_parameters(pid_t *pid, const pid_parameters_t *parameters) {
    if (parameters->kp < 0 || parameters->ki < 0|| parameters->kd < 0) {
        return;
    }
    float sample_time_s = (float)pid->sample_time / 1000;
    pid->parameters.kp = parameters->kp;
    pid->parameters.ki = parameters->ki * sample_time_s;
    pid->parameters.kd = parameters->kd / sample_time_s;

    if (pid->direction == DB_PID_DIRECTION_REVERSED) {
        pid->parameters.kp = (0 - pid->parameters.kp);
        pid->parameters.ki = (0 - pid->parameters.ki);
        pid->parameters.kd = (0 - pid->parameters.kd);
    }
}

void db_pid_set_sample_time(pid_t *pid, uint32_t sample_time) {
    assert(pid->sample_time != 0 && sample_time != 0);
    float ratio = (float)sample_time / pid->sample_time;
    pid->parameters.ki *= ratio;
    pid->parameters.kd /= ratio;
    pid->sample_time = sample_time;
}

void db_pid_set_output_limits(pid_t *pid, float output_min, float output_max) {
    assert (output_min < output_max);
    pid->output_min = output_min;
    pid->output_max = output_max;

    if (pid->mode == DB_PID_MODE_AUTO) {
        if (pid->output > output_max) {
            pid->output = output_max;
        }
        else if (pid->output < output_min) {
            pid->output = output_min;
        }
        if (pid->state.output_sum > output_max) {
            pid->state.output_sum = output_max;
        }
        else if (pid->state.output_sum < output_min) {
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
                    pid->parameters.kp, pid->parameters.ki, pid->parameters.kd,
                    pid->sample_time, pid->output_min, pid->output_max,
                    pid->mode, pid->direction);
    }
}

void db_pid_set_direction(pid_t *pid, pid_direction_t direction) {
    if (direction == pid->direction) {
        return;
    }

    if (pid->mode == DB_PID_MODE_AUTO && pid->direction != direction) {
        pid->parameters.kp = (0 - pid->parameters.kp);
        pid->parameters.ki = (0 - pid->parameters.ki);
        pid->parameters.kd = (0 - pid->parameters.kd);
    }
    pid->direction = direction;
}
