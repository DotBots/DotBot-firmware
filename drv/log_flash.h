#ifndef __LOG_FLASH_H
#define __LOG_FLASH_H

/**
 * @file log_flash.h
 * @addtogroup DRV
 *
 * @brief  Cross-platform declaration "log_flash" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdlib.h>
#include <stdint.h>

//=========================== defines ==========================================

#define DB_LOG_MAGIC 0xDBDB

typedef enum {
    LOG_DATA_DOTBOT = 0
} db_log_data_type_t;

typedef struct __attribute__((packed)) {
    uint16_t           magic;
    uint8_t            version;
    db_log_data_type_t log_type;
} db_log_header_t;

typedef struct __attribute__((packed)) {
    int32_t  direction;
    uint32_t pos_x;
    uint32_t pos_y;
    uint16_t next_waypoint_idx;
    uint32_t distance_to_target;
    int16_t  angle_to_target;
    int16_t  error_angle;
    int16_t  angular_speed;
    int16_t  left_speed;
    int16_t  right_speed;
} db_log_dotbot_data_t;

//=========================== public ===========================================

/**
 * @brief Initialize the log_flash driver
 */
void db_log_flash_init(db_log_data_type_t type);

/**
 * @brief Write log data to flash
 * @param data  Pointer to the  log data to write
 * @param len   Length of log data to write
 */
void db_log_flash_write(const void *data, size_t len);

#endif
