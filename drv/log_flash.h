#ifndef __LOG_FLASH_H
#define __LOG_FLASH_H

/**
 * @defgroup    drv_log_flash   Log data to flash
 * @ingroup     drv
 * @brief       Helper library for logging data to flash
 *
 * Depends on the @ref bsp_nvmc module
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdlib.h>
#include <stdint.h>

//=========================== defines ==========================================

#define DB_LOG_MAGIC 0xDBDB  ///< Magic number for log data

/// Log data type
typedef enum {
    LOG_DATA_DOTBOT = 0  ///< DotBot data
} db_log_data_type_t;

/// Log header
typedef struct __attribute__((packed)) {
    uint16_t           magic;     ///< Magic number
    uint8_t            version;   ///< Log data version
    db_log_data_type_t log_type;  ///< Log data type
} db_log_header_t;

///< Log data for the DotBot autonomous control loop
typedef struct __attribute__((packed)) {
    int32_t  direction;           ///< Direction of the DotBot
    uint32_t pos_x;               ///< X position of the DotBot
    uint32_t pos_y;               ///< Y position of the DotBot
    uint16_t next_waypoint_idx;   ///< Index of the next waypoint
    uint32_t distance_to_target;  ///< Distance to the target
    int16_t  angle_to_target;     ///< Angle to the target
    int16_t  error_angle;         ///< Error angle
    int16_t  angular_speed;       ///< Angular speed
    int16_t  left_speed;          ///< Left wheel speed
    int16_t  right_speed;         ///< Right wheel speed
} db_log_dotbot_data_t;

//=========================== public ===========================================

/**
 * @brief Initialize the log_flash driver
 * @param[in]   type    Type of log data to log to flash
 */
void db_log_flash_init(db_log_data_type_t type);

/**
 * @brief Write log data to flash
 * @param[in]   data    Pointer to the log data to write
 * @param[in]   len     Length of log data to write
 */
void db_log_flash_write(const void *data, size_t len);

#endif
