#ifndef __RPM_H
#define __RPM_H

/**
 * @defgroup    bsp_rpm     RPM driver
 * @ingroup     bsp
 * @brief       Control the RPM driver (dotbot v1 only)
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>

//=========================== defines ==========================================

/**
 * Structure containing different values computed by the rpm driver
 */
typedef struct {
    uint32_t rpm;    ///<  Rotation per minute
    uint32_t rps;    ///<  Rotation per second
    float    speed;  ///<  Speed in cm/s
} rpm_side_values_t;

/**
 * Structure containing different rpm value of each motor (left and right)
 */
typedef struct {
    rpm_side_values_t left;   ///< rpm values of the left motor
    rpm_side_values_t right;  ///< rpm values of the right motor
} rpm_values_t;

//=========================== prototypes =======================================

/**
 * @brief Initalize the RPM driver
 *
 * 2 GPIOTE input pins with one timer each. Each time a magnet comes in front of the magnetic encoder,
 * GPIOTE event is triggered which clears the timer ticks counter, for each side (left and right).
 * The speed/rpm are computed by the user code on demand by capturing the timer current count, reading
 * the timer CC register and clearing the timer count. Computations are done with the `ME_TICK_TO_*`
 * constants.
 */
void db_rpm_init(void);

/**
 * Get values (rpm, rps, speed) measured by the rpm driver
 *
 * @param[out] values   A pointer to the struct handling all values
 */
void db_rpm_get_values(rpm_values_t *values);

#endif
