#ifndef __PARTITION_H
#define __PARTITION_H

/**
 * @defgroup    bsp_partition   Partition table support
 * @ingroup     bsp
 * @brief       Manipulate the partition table
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>

#include <nrf.h>

#define DB_PARTITIONS_TABLE_MAGIC (0xD07B0723UL)  ///< Magic number used at the beginning of the partition table
#define DB_PARTITIONS_MAX_COUNT   (4U)            ///< Maximum number of available partitions

/// Partition table entry
typedef struct {
    uint32_t address;  ///< Start address of a partition
    uint32_t size;     ///< Size of the partition
} db_partition_t;

/// Partition table
typedef struct {
    uint32_t       magic;                                ///< Magic number
    uint32_t       length;                               ///< Current number of partitions in the table
    uint32_t       active_image;                         ///< Active image to boot on
    db_partition_t partitions[DB_PARTITIONS_MAX_COUNT];  ///< List of partitions
} db_partitions_table_t;

/**
 * @brief Read partition table on flash
 * @param[out]  partitions      Pointer to the runtime partition table
 */
void db_read_partitions_table(db_partitions_table_t *partitions);

/**
 * @brief Write partition table on flash
 * @param[in]   partitions      Pointer to the runtime partition table
 */
void db_write_partitions_table(const db_partitions_table_t *partitions);

#endif  // __PARTITION_H
