#ifndef __PARTITION_H
#define __PARTITION_H

/**
 * @file partition.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "partition" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include <string.h>

#include <nrf.h>

#define DB_PARTITIONS_TABLE_MAGIC (0xD07B0723UL)  ///< Magic number used at the beginning of the partition table
#define DB_PARTITIONS_MAX_COUNT   (4U)            ///< Maximum number of available partitions

typedef struct {
    uint32_t address;  ///< Start address of a partition
    uint32_t size;     ///< Size of the partition
} db_partition_t;

typedef struct {
    uint32_t       magic;                                ///< Magic number
    uint32_t       length;                               ///< Current number of partitions in the table
    uint32_t       active_image;                         ///< Active image to boot on
    db_partition_t partitions[DB_PARTITIONS_MAX_COUNT];  ///< List of partitions
} db_partitions_table_t;

/**
 * @brief Read partition table on flash
 * @param data  Pointer to the runtime partition table
 */
void db_read_partitions_table(db_partitions_table_t *partitions);

/**
 * @brief Write partition table on flash
 * @param data  Pointer to the runtime partition table
 */
void db_write_partitions_table(const db_partitions_table_t *partitions);

#endif  // __PARTITION_H
