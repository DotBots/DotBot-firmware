#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/**
 * @defgroup    drv_protocol    DotBot protocol implementation
 * @ingroup     drv
 * @brief       Definitions and implementations of the DotBot protocol
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//=========================== defines ==========================================

#define DB_FIRMWARE_VERSION  (1)                   ///< Version of the firmware
#define DB_BROADCAST_ADDRESS 0xffffffffffffffffUL  ///< Broadcast address
#define DB_GATEWAY_ADDRESS   0x0000000000000000UL  ///< Gateway address
#define DB_MAX_WAYPOINTS     (16)                  ///< Max number of waypoints

/// Command type
typedef enum {
    DB_PROTOCOL_CMD_MOVE_RAW       = 0,   ///< Move raw command type
    DB_PROTOCOL_CMD_RGB_LED        = 1,   ///< RGB LED command type
    DB_PROTOCOL_LH2_RAW_DATA       = 2,   ///< Lighthouse 2 raw data
    DB_PROTOCOL_LH2_LOCATION       = 3,   ///< Lighthouse processed locations
    DB_PROTOCOL_ADVERTISEMENT      = 4,   ///< DotBot advertisements
    DB_PROTOCOL_GPS_LOCATION       = 5,   ///< GPS data from SailBot
    DB_PROTOCOL_DOTBOT_DATA        = 6,   ///< DotBot specific data (for now location and direction)
    DB_PROTOCOL_CONTROL_MODE       = 7,   ///< Robot remote control mode (automatic or manual)
    DB_PROTOCOL_LH2_WAYPOINTS      = 8,   ///< List of LH2 waypoints to follow
    DB_PROTOCOL_GPS_WAYPOINTS      = 9,   ///< List of GPS waypoints to follow
    DB_PROTOCOL_SAILBOT_DATA       = 10,  ///< SailBot specific data (for now GPS and direction)
    DB_PROTOCOL_CMD_XGO_ACTION     = 11,  ///< XGO action command
    DB_PROTOCOL_LH2_PROCESSED_DATA = 12,  ///< Lighthouse 2 data processed at the DotBot
    DB_PROTOCOL_LH2_CALIBRATION    = 14,  ///< Lighthouse 2 homography matrix after calibration
} protocol_data_type_t;

/// Protocol packet type
typedef enum {
    DB_PACKET_BEACON            = 1,    ///< Beacon packet
    DB_PACKET_JOIN_REQUEST      = 2,    ///< Join request packet
    DB_PACKET_JOIN_RESPONSE     = 4,    ///< Join response packet
    DB_PACKET_KEEP_ALIVE        = 6,    ///< Leave packet
    DB_PACKET_DATA              = 16,   ///< Data packet
    DB_PACKET_TDMA_UPDATE_TABLE = 128,  ///< TDMA table update packet
    DB_PACKET_TDMA_SYNC_FRAME   = 129,  ///< TDMA sync frame packet
    DB_PACKET_TDMA_KEEP_ALIVE   = 130,  ///< TDMA keep alive packet
} packet_type_t;

/// Application type
typedef enum {
    DotBot        = 0,  ///< DotBot application
    SailBot       = 1,  ///< SailBot application
    FreeBot       = 2,  ///< FreeBot application
    XGO           = 3,  ///< XGO application
    LH2_mini_mote = 4,  ///< LH2 mini mote application
} application_type_t;

/// Control mode
typedef enum {
    ControlManual = 0,  ///< Manual mode
    ControlAuto   = 1,  ///< Automatic mode
} protocol_control_mode_t;

/// DotBot protocol header
typedef struct __attribute__((packed)) {
    uint8_t       version;      ///< Version of the firmware
    packet_type_t packet_type;  ///< Type of packet
    uint64_t      dst;          ///< Destination address of this packet
    uint64_t      src;          ///< Source address of this packet
} protocol_header_t;

/// DotBot protocol move raw command
typedef struct __attribute__((packed)) {
    int8_t left_x;   ///< Horizontal coordinate for left side
    int8_t left_y;   ///< Vertical coordinate for left side
    int8_t right_x;  ///< Horizontal coordinate for right side
    int8_t right_y;  ///< Vertical coordinate for right side
} protocol_move_raw_command_t;

/// DotBot protocol RGB LED command
typedef struct __attribute__((packed)) {
    uint8_t r;  ///< Red component value
    uint8_t g;  ///< Green component value
    uint8_t b;  ///< Blue component value
} protocol_rgbled_command_t;

/// DotBot protocol LH2 computed location
typedef struct __attribute__((packed)) {
    uint32_t x;  ///< X coordinate, multiplied by 1e6
    uint32_t y;  ///< Y coordinate, multiplied by 1e6
    uint32_t z;  ///< Z coordinate, multiplied by 1e6
} protocol_lh2_location_t;

/// DotBot protocol LH2 waypoints
typedef struct __attribute__((packed)) {
    uint8_t                 length;                    ///< Number of waypoints
    protocol_lh2_location_t points[DB_MAX_WAYPOINTS];  ///< Array containing a list of lh2 point coordinates
} protocol_lh2_waypoints_t;

typedef struct __attribute__((packed)) {
    uint8_t basestation_index;        ///< which LH basestation is this homography for?
    int32_t homography_matrix[3][3];  ///< homography matrix, each element multiplied by 1e6
} protocol_lh2_homography_t;

/// DotBot protocol GPS coordinates
typedef struct __attribute__((packed)) {
    int32_t latitude;   ///< Latitude, multiplied by 1e6
    int32_t longitude;  ///< Longitude, multiplied by 1e6
} protocol_gps_coordinate_t;

/// DotBot protocol GPS waypoints
typedef struct __attribute__((packed)) {
    uint8_t                   length;                         ///< Number of waypoints
    protocol_gps_coordinate_t coordinates[DB_MAX_WAYPOINTS];  ///< Array containing a list of GPS coordinates
} protocol_gps_waypoints_t;

/// LH2 process data compressed for sending over radio.
typedef struct __attribute__((packed)) {
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    uint32_t lfsr_location;        ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
    uint32_t timestamp_us;         ///< How many microseconds passed since the sample was taken
} protocol_lh2_processed_packet_t;

/// DotBot protocol TDMA table update [all units are in microseconds]
typedef struct __attribute__((packed)) {
    uint32_t frame_period;       ///< duration of a full TDMA frame
    uint32_t rx_start;           ///< start to listen for packets
    uint16_t rx_duration;        ///< duration of the RX period
    uint32_t tx_start;           ///< start of slot for transmission
    uint16_t tx_duration;        ///< duration of the TX period
    uint32_t next_period_start;  ///< time until the start of the next TDMA frame
} protocol_tdma_table_t;

/// DotBot protocol sync messages marks the start of a TDMA frame [all units are in microseconds]
typedef struct __attribute__((packed)) {
    uint32_t frame_period;  ///< duration of a full TDMA frame
} protocol_sync_frame_t;

//=========================== public ===========================================

/**
 * @brief   Write the protocol header in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_header_to_buffer(uint8_t *buffer, uint64_t dst);

/**
 * @brief   Write a TDMA keep alive packet in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_tdma_keep_alive_to_buffer(uint8_t *buffer, uint64_t dst);

/**
 * @brief   Write a TDMA table update in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   tdma_table  Pointer to the TDMA table
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_tdma_table_update_to_buffer(uint8_t *buffer, uint64_t dst, protocol_tdma_table_t *tdma_table);

/**
 * @brief   Write a TDMA sync frame in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   sync_frame  Pointer to the sync frame
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_tdma_sync_frame_to_buffer(uint8_t *buffer, uint64_t dst, protocol_sync_frame_t *sync_frame);

/**
 * @brief   Write an application advertizement packet in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   application Type of application advertized
 * @param[in]   calibrated  Whether the device LH2 is calibrated (true) or not (false)
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_advertizement_to_buffer(uint8_t *buffer, uint64_t dst, application_type_t application, bool calibrated);

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   command     Pointer to the move raw command
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_cmd_move_raw_to_buffer(uint8_t *buffer, uint64_t dst, protocol_move_raw_command_t *command);

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   command     Pointer to the rgbled command
 *
 * @return                  Number of bytes written in the buffer
 */
size_t db_protocol_cmd_rgbled_to_buffer(uint8_t *buffer, uint64_t dst, protocol_rgbled_command_t *command);

#endif
