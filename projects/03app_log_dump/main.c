/**
 * @file main.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This application is used to read and print the logs available on flash
 *
 * @copyright Inria, 2023
 *
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include <nrf.h>

#include "nvmc.h"
#include "timer_hf.h"

#include "log_flash.h"
#include "protocol.h"

//=========================== defines ==========================================

#define DB_LOG_FLASH_START ((DB_FLASH_PAGE_SIZE * DB_FLASH_PAGE_NUM) / 2)

//=========================== variables ========================================

static uint32_t *_read_address = (uint32_t *)(DB_LOG_FLASH_START + DB_FLASH_OFFSET);

//=========================== main =============================================

int main(void) {
    db_timer_hf_init();

    db_log_header_t header;
    db_nvmc_read(&header, _read_address++, sizeof(db_log_header_t));

    if (header.magic != DB_LOG_MAGIC) {
        puts("Wrong magic number, invalid data");
        goto loop;
    }

    if (header.version != DB_FIRMWARE_VERSION) {
        puts("Incompatible firmware version");
        goto loop;
    }

    puts("Waiting for 's' input to start reading the logs");
    char c = '\0';
    do {
        c = getchar();
    } while (c != 's');

    puts("START");

    switch (header.log_type) {
        case LOG_DATA_DOTBOT:
            puts(
                "timestamp;direction;pos_x;pos_y;next_waypoint_idx;"
                "distance_to_target;angle_to_target;error_angle;"
                "angular_speed;left_speed;right_speed");
            break;
        default:
            puts("Unsupported log data type");
            goto loop;
    }

    db_timer_hf_delay_ms(20);
    bool read = true;
    while (read) {
        switch (header.log_type) {
            case LOG_DATA_DOTBOT:
            {
                uint32_t timestamp = 0;
                db_nvmc_read(&timestamp, _read_address++, sizeof(uint32_t));
                if (timestamp == 0xffffffff) {
                    puts("STOP (end of logs reached)");
                    goto loop;
                }
                db_log_dotbot_data_t data;
                db_nvmc_read(&data, _read_address, sizeof(db_log_dotbot_data_t));
                printf(
                    "%lu;%i;%lu;%lu;%u;%lu;%i;%i;%i;%i;%i\n",
                    timestamp, data.direction, data.pos_x, data.pos_y,
                    data.next_waypoint_idx, data.distance_to_target,
                    data.angle_to_target, data.error_angle, data.angular_speed,
                    data.left_speed, data.right_speed);
                _read_address += (sizeof(db_log_dotbot_data_t) >> 2);
            } break;
        }
        db_timer_hf_delay_ms(20);
        read = ((uint32_t)_read_address < DB_FLASH_PAGE_NUM * DB_FLASH_PAGE_SIZE);
        if (!read) {
            puts("STOP (end of flash reached)");
        }
    }

loop:
    while (1) {
        __WFE();
    };
}
