#ifndef __MOTORS_H
#define __MOTORS_H

/**
 * @file motors.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "motors" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>

//=========================== prototypes ======================================

//============================= defines =======================================

#define PIN_INVALID -1

typedef struct {
    int16_t pin[4];
    int16_t port[4];
    uint16_t mtop;
    uint16_t pwm_seq[4];
} motors_vars_t;

//=========================== public ======================================

void db_motors_init(void);
void db_pwm_init(motors_vars_t *ctx);
void db_motors_set_pwm_length(motors_vars_t *ctx, uint8_t channel, uint16_t pwm_length);
void db_motors_set_speed(int16_t l_speed, int16_t r_speed);

#endif
