#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/**
 * @file procotol.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "protocol" driver module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

#define DB_FIRMWARE_VERSION  (7)                   ///< Version of the firmware
#define DB_SWARM_ID          (0x0000)              ///< Default swarm ID
#define DB_BROADCAST_ADDRESS 0xffffffffffffffffUL  ///< Broadcast address
#define DB_GATEWAY_ADDRESS   0x0000000000000000UL  ///< Gateway address
#define DB_MAX_WAYPOINTS     (16)                  ///< Max number of waypoints

typedef enum {
    DB_PROTOCOL_CMD_MOVE_RAW  = 0,   ///< Move raw command type
    DB_PROTOCOL_CMD_RGB_LED   = 1,   ///< RGB LED command type
    DB_PROTOCOL_LH2_RAW_DATA  = 2,   ///< Lighthouse 2 raw data
    DB_PROTOCOL_LH2_LOCATION  = 3,   ///< Lighthouse processed locations
    DB_PROTOCOL_ADVERTISEMENT = 4,   ///< DotBot advertisements
    DB_PROTOCOL_GPS_LOCATION  = 5,   ///< GPS data from SailBot
    DB_PROTOCOL_DOTBOT_DATA   = 6,   ///< DotBot specific data (for now location and direction)
    DB_PROTOCOL_CONTROL_MODE  = 7,   ///< Robot remote control mode (automatic or manual)
    DB_PROTOCOL_LH2_WAYPOINTS = 8,   ///< List of LH2 waypoints to follow
    DB_PROTOCOL_GPS_WAYPOINTS = 9,   ///< List of GPS waypoints to follow
    DB_PROTOCOL_SAILBOT_DATA  = 10,  ///< SailBot specific data (for now GPS and direction)
    DB_PROTOCOL_EKF_DEBUG     = 11,  ///< Information on the internal variables of the ekf to better debug it.
} command_type_t;

typedef enum {
    DotBot  = 0,  ///< DotBot application
    SailBot = 1,  ///< SailBot application
} application_type_t;

typedef enum {
    ControlManual = 0,  ///< Manual mode
    ControlAuto   = 1,  ///< Automatic mode
} protocol_control_mode_t;

typedef struct __attribute__((packed)) {
    uint64_t           dst;          ///< Destination address of this packet
    uint64_t           src;          ///< Source address of this packet
    uint16_t           swarm_id;     ///< Swarm ID
    application_type_t application;  ///< Application type
    uint8_t            version;      ///< Version of the firmware
    command_type_t     type;         ///< Type of command following this header
} protocol_header_t;

typedef struct __attribute__((packed)) {
    int8_t left_x;   ///< Horizontal coordinate for left side
    int8_t left_y;   ///< Vertical coordinate for left side
    int8_t right_x;  ///< Horizontal coordinate for right side
    int8_t right_y;  ///< Vertical coordinate for right side
} protocol_move_raw_command_t;

typedef struct __attribute__((packed)) {
    uint8_t r;  ///< Red component value
    uint8_t g;  ///< Green component value
    uint8_t b;  ///< Blue component value
} protocol_rgbled_command_t;

typedef struct __attribute__((packed)) {
    uint32_t x;  ///< X coordinate, multiplied by 1e6
    uint32_t y;  ///< Y coordinate, multiplied by 1e6
    uint32_t z;  ///< Z coordinate, multiplied by 1e6
} protocol_lh2_location_t;

typedef struct __attribute__((packed)) {
    uint8_t                 length;                    ///< Number of waypoints
    protocol_lh2_location_t points[DB_MAX_WAYPOINTS];  ///< Array containing a list of lh2 point coordinates
} protocol_lh2_waypoints_t;

typedef struct __attribute__((packed)) {
    int32_t latitude;   ///< Latitude, multiplied by 1e6
    int32_t longitude;  ///< Longitude, multiplied by 1e6
} protocol_gps_coordinate_t;

typedef struct __attribute__((packed)) {
    uint8_t                   length;                         ///< Number of waypoints
    protocol_gps_coordinate_t coordinates[DB_MAX_WAYPOINTS];  ///< Array containing a list of GPS coordinates
} protocol_gps_waypoints_t;

typedef struct __attribute__((packed)) {
    int32_t x;      ///< X coordinate, in [um]
    int32_t y;      ///< Y coordinate, in [um]
    int32_t theta;  ///< Angle of orientation, in [mili degrees], from -pi to +pi
    uint16_t V;     ///< Forward speed, in [mm/s]
    int32_t w;      ///< Angular speed. in [mili degrees per second].
    int16_t angle_to_target;  ///< Angle between the dotbot and the waypoint. , in [mili degrees], from -pi to +pi.
} protocol_ekf_debug_t;

//=========================== public ===========================================

/**
 * @brief   Write the protocol header in a buffer
 *
 * @param[out]  buffer          Bytes array to write to
 * @param[in]   dst             Destination address written in the header
 * @param[in]   application     Application type that relates to this header
 * @param[in]   command_type    Command type that follows this header
 */
void db_protocol_header_to_buffer(uint8_t *buffer, uint64_t dst, application_type_t application, command_type_t command_type);

/**
 * @brief   Write a move raw command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   application Application type that relates to this command
 * @param[in]   command     Pointer to the move raw command
 */
void db_protocol_cmd_move_raw_to_buffer(uint8_t *buffer, uint64_t dst, application_type_t application, protocol_move_raw_command_t *command);

/**
 * @brief   Write an rgbled command in a buffer
 *
 * @param[out]  buffer      Bytes array to write to
 * @param[in]   dst         Destination address written in the header
 * @param[in]   application Application type that relates to this command
 * @param[in]   command     Pointer to the rgbled command
 */
void db_protocol_cmd_rgbled_to_buffer(uint8_t *buffer, uint64_t dst, application_type_t application, protocol_rgbled_command_t *command);

#endif
