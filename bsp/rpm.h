#ifndef __RPM_H
#define __RPM_H

/**
 * @file rpm.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "rpm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

/**
 * Structure containing different values computed by the rpm driver
 */
typedef struct {
    uint32_t rpm;               /**<  Rotation per minute */
    uint32_t rps;               /**<  Rotation per second */
    uint32_t speed;             /**<  Speed in cm/s */
} rpm_side_values_t;

/**
 * Structure containing different rpm value of each motor (left and right)
 */
typedef struct {
    rpm_side_values_t left;     /**< rpm values of the left motor */
    rpm_side_values_t right;    /**< rpm values of the right motor*/
} rpm_values_t;

//=========================== prototypes =======================================

/**
 * Initialize the rpm driver
 */
void db_rpm_init(void);

/**
 * Get values (rpm, rps, speed) measured by the rpm driver
 *
 * @param[out] values   A pointer to the struct handling all values
 */
void db_rpm_get_values(rpm_values_t *values);

#endif
