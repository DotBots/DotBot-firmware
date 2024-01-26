/**
 * @file
 * @defgroup project_sailbot    SailBot application
 * @ingroup projects
 * @brief This is the radio-controlled SailBot app
 *
 * Load this program on your board. Now the SailBot can be remotely controlled
 * from a nearby nRF52840-DK.
 *
 *
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>
#include <string.h>

// Include BSP packages
#include "device.h"
#include "radio.h"
#include "servos.h"
#include "gps.h"
#include "lis2mdl.h"
#include "lsm6ds.h"
#include "imu.h"
#include "protocol.h"
#include "timer.h"
#include "timer_hf.h"
#include "gpio.h"
#include "assert.h"
#include "math.h"

//=========================== defines =========================================

#define CONTROL_LOOP_PERIOD_MS      (1000)  ///< control loop period
#define WAYPOINT_DISTANCE_THRESHOLD (10)    ///< in meters

#define SAIL_TRIM_ANGLE_UNIT_STEP (10)     //< unit step increase/decrease when trimming the sails
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define TIMEOUT_CHECK_DELAY_MS    (200)    ///< 200 ms delay between packet received timeout checks
#define ADVERTISEMENT_PERIOD_MS   (500)    ///< send an advertisement every 500 ms
#define DB_BUFFER_MAX_BYTES       (64U)    ///< Max bytes in UART receive buffer

#define CONST_EARTH_RADIUS_KM          (6371.0F)               // 6371 km
#define CONST_COS_PHI_0_INRIA_PARIS    (0.658139837F)          // cos(0.85245092073) where 0.85245092073 represent radians latitude of Inria Paris
#define CONST_COS_PHI_0_UFF            (0.92114562603F)        // cos(0.3997826147759739) where 0.3997826147759739 represents radians latitude of UFF
#define CONST_METRO_LIBERTE_LAT        (48.825908013055255F)   // coordinates of Metro Liberte subway station in Paris
#define CONST_METRO_LIBERTE_LONG       (2.4064333460310094F)   // coordinates of Metro Liberte subway station in Paris
#define CONST_NOTRE_DAME_DE_PARIS_LAT  (48.85443748F)          // coordinates of Notre Dame de Paris
#define CONST_NOTRE_DAME_DE_PARIS_LONG (2.350162268F)          // coordinates of Notre Dame de Paris
#define CONST_METRO_BERCY_LAT          (48.840280818580354F)   // coordinates of Metro Bercy subway station in Paris
#define CONST_METRO_BERCY_LONG         (2.379762322245707F)    // coordinates of Metro Bercy subway station in Paris
#define CONST_ESCOLA_NAVAL_LAT         (-22.913357509339527F)  // coordinates of Escola Naval in Rio
#define CONST_ESCOLA_NAVAL_LONG        (-43.15753771789026F)   // coordinates of Escola Naval in Rio

// user-select defines
// make sure CONST_COS_PHI_0 latitude is approximately at the middle of the map
#define CONST_ORIGIN_COORD_SYSTEM_LAT  CONST_METRO_LIBERTE_LAT
#define CONST_ORIGIN_COORD_SYSTEM_LONG CONST_METRO_LIBERTE_LONG
#define CONST_COS_PHI_0                CONST_COS_PHI_0_INRIA_PARIS

typedef struct {
    float x;
    float y;
} cartesian_coordinate_t;

typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    int8_t                   sail_trim;                          ///< Last angle of the servo controlling sail trim
    protocol_gps_waypoints_t waypoints;                          ///< List of waypoints
    uint32_t                 waypoints_threshold;                ///< Distance threshold to next waypoint
    uint8_t                  next_waypoint_idx;                  ///< Index of next waypoint to reach
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    bool                     advertise;                          ///< Flag used to indicate that advertisement needs to be sent
    bool                     send_log_data;                      ///< Flag used to indicate that log data needs to be sent
    bool                     autonomous_operation;               ///< Flag used to enable/disable autonomous operation
    bool                     radio_override;                     ///< Flag used to override autonomous operation when radio-controlled
    lis2mdl_compass_data_t   last_magnetometer;                  ///< Last reading of the magnetometer
    lsm6ds_acc_data_t        last_accelerometer;                 ///< Last reading of the accelerometer
    nmea_gprmc_t            *last_gps_data;
    uint16_t                 last_heading;
} sailbot_vars_t;

//=========================== variables =========================================

static const gpio_t   _led1_pin     = { .pin = 15, .port = 0 };
static sailbot_vars_t _sailbot_vars = { 0 };

//=========================== prototypes =========================================

void          radio_callback(uint8_t *packet, uint8_t length);
void          control_loop_callback(void);
static void   convert_geographical_to_cartesian(cartesian_coordinate_t *out, const protocol_gps_coordinate_t *in);
static float  _distance(const cartesian_coordinate_t *pos1, const cartesian_coordinate_t *pos2);
static float  calculate_error(float heading, float bearing);
static int8_t map_error_to_rudder_angle(float error);
static void   _timeout_check(void);
static void   _advertise(void);
static void   _send_gps_data(const nmea_gprmc_t *data, uint16_t heading);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Turn ON the LED1
    NRF_P0->DIRSET = 1 << _led1_pin.pin;  // set pin as output
    NRF_P0->OUTSET = 0 << _led1_pin.pin;  // set pin LOW

    _sailbot_vars.autonomous_operation = false;
    _sailbot_vars.radio_override       = false;
    _sailbot_vars.advertise            = false;
    _sailbot_vars.send_log_data        = false;
    _sailbot_vars.sail_trim            = 50;

    // Initialize the protocol
    db_protocol_init();

    // Configure Radio as a receiver
    db_radio_init(&radio_callback, DB_RADIO_BLE_LR125Kbit);  // Set the callback function.
    db_radio_set_frequency(8);                               // Set the RX frequency to 2408 MHz.
    db_radio_rx();                                           // Start receiving packets.

    // Init the IMU chips
    imu_init(NULL, NULL);

    // Configure Motors
    servos_init();
    servos_set(0, _sailbot_vars.sail_trim);

    // init the timers
    db_timer_init();

    // Configure GPS without callback
    gps_init(NULL);

    // set timer callbacks
    db_timer_set_periodic_ms(0, TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(1, ADVERTISEMENT_PERIOD_MS, &_advertise);
    db_timer_set_periodic_ms(2, CONTROL_LOOP_PERIOD_MS, &control_loop_callback);

    // Wait for radio packets to arrive
    while (1) {
        // processor idle until an interrupt occurs and is handled
        // if IMU data is ready, trigger the I2C transfer from outside the interrupt context
        if (lis2mdl_data_ready()) {
            lis2mdl_read_magnetometer(&_sailbot_vars.last_magnetometer);
        }
        if (lsm6ds_data_ready()) {
            lsm6ds_read_accelerometer(&_sailbot_vars.last_accelerometer);
        }
        if (_sailbot_vars.advertise) {
            db_protocol_header_to_buffer(_sailbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, SailBot, DB_PROTOCOL_ADVERTISEMENT);
            size_t length = sizeof(protocol_header_t);
            db_radio_disable();
            db_radio_tx(_sailbot_vars.radio_buffer, length);
            _sailbot_vars.advertise = false;
        }
        if (_sailbot_vars.send_log_data) {
            _send_gps_data(_sailbot_vars.last_gps_data, _sailbot_vars.last_heading);
            _sailbot_vars.send_log_data = false;
        }
        __WFE();
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== functions =========================================

/**
 *  @brief Callback function to process received packets
 *
 * This function gets called each time a packet is received.
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void radio_callback(uint8_t *packet, uint8_t length) {
    (void)length;
    uint8_t           *ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // timestamp the arrival of the packet
    _sailbot_vars.ts_last_packet_received = db_timer_ticks();

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
        return;
    }

    // Check version is compatible
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Check application is compatible
    if (header->application != SailBot) {
        return;
    }

    // Process the command received
    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    switch (header->type) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;

            _sailbot_vars.radio_override = true;

            if (command->right_y > 0 && ((int16_t)_sailbot_vars.sail_trim + SAIL_TRIM_ANGLE_UNIT_STEP < 127)) {
                _sailbot_vars.sail_trim += SAIL_TRIM_ANGLE_UNIT_STEP;
            } else if (command->right_y < 0 && ((int16_t)_sailbot_vars.sail_trim - SAIL_TRIM_ANGLE_UNIT_STEP > 0)) {
                _sailbot_vars.sail_trim -= SAIL_TRIM_ANGLE_UNIT_STEP;
            }
            // set the servos
            servos_set(command->left_x, _sailbot_vars.sail_trim);
        } break;
        case DB_PROTOCOL_GPS_WAYPOINTS:
        {
            servos_set(0, _sailbot_vars.sail_trim);
            _sailbot_vars.radio_override       = false;
            _sailbot_vars.autonomous_operation = false;
            _sailbot_vars.waypoints.length     = (uint8_t)*cmd_ptr++;
            _sailbot_vars.waypoints_threshold  = (uint32_t)((uint8_t)*cmd_ptr++);
            memcpy(&_sailbot_vars.waypoints.coordinates, cmd_ptr, _sailbot_vars.waypoints.length * sizeof(protocol_gps_coordinate_t));
            _sailbot_vars.next_waypoint_idx = 0;
            if (_sailbot_vars.waypoints.length > 0) {
                _sailbot_vars.autonomous_operation = true;
            }
        } break;
        default:
            break;
    }
}

void control_loop_callback(void) {
    // Read the GPS
    _sailbot_vars.last_gps_data = gps_last_known_position();

    if (!_sailbot_vars.last_gps_data->valid) {
        return;
    }

    // get tilt-compensated heading
    float heading              = imu_calculate_tilt_compensated_heading(&_sailbot_vars.last_magnetometer, &_sailbot_vars.last_accelerometer);
    _sailbot_vars.last_heading = (uint16_t)(heading * 180 / M_PI);

    _sailbot_vars.send_log_data = true;

    if (!_sailbot_vars.autonomous_operation) {
        // Do nothing if not in autonomous operation
        return;
    }

    if (_sailbot_vars.next_waypoint_idx >= _sailbot_vars.waypoints.length) {
        // Reset the rudder and sail when the last waypoint is reached
        _sailbot_vars.autonomous_operation = false;
        servos_set(0, _sailbot_vars.sail_trim);
        return;
    }

    protocol_gps_coordinate_t current_position_gps = { 0, 0 };
    cartesian_coordinate_t    target               = { 0, 0 };
    cartesian_coordinate_t    position             = { 0, 0 };
    float                     theta                = 0;
    float                     error                = 0;
    int8_t                    rudder_angle         = 0;

    // convert the next_waypoint to local coordinate system (and copy to stack to avoid concurrency issues)
    convert_geographical_to_cartesian(&target, &_sailbot_vars.waypoints.coordinates[_sailbot_vars.next_waypoint_idx]);

    // save the current GPS position on stack to avoid concurrency issues between control_loop_callback() and the GPS module
    current_position_gps.latitude  = (int32_t)(_sailbot_vars.last_gps_data->latitude * 1e6);
    current_position_gps.longitude = (int32_t)(_sailbot_vars.last_gps_data->longitude * 1e6);

    // convert geographical data given by GPS to our local coordinate system
    convert_geographical_to_cartesian(&position, &current_position_gps);

    float distance_to_target = _distance(&target, &position);

    // Check the next waypoint was reached, if yes increase the waypoint index and return
    if (distance_to_target < _sailbot_vars.waypoints_threshold) {
        _sailbot_vars.next_waypoint_idx++;
        return;
    }

    // calculate the angle theta, which is the angle between myself and the waypoint relative to the y axis
    theta = atan2f(target.x - position.x, target.y - position.y);  // north clockwise convention
    if (theta < 0) {
        theta += M_PI * 2;
    }

    // calculate the error
    error = calculate_error(heading, theta);

    // convert error in radians to rudder angle
    rudder_angle = map_error_to_rudder_angle(error);

    // set the rudder servo
    if (!_sailbot_vars.radio_override) {
        servos_rudder_turn(rudder_angle);
    }
}

static void _send_gps_data(const nmea_gprmc_t *data, uint16_t heading) {
    int32_t latitude  = (int32_t)(data->latitude * 1e6);
    int32_t longitude = (int32_t)(data->longitude * 1e6);

    db_protocol_header_to_buffer(_sailbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, SailBot, DB_PROTOCOL_SAILBOT_DATA);

    memcpy(_sailbot_vars.radio_buffer + sizeof(protocol_header_t), &heading, sizeof(uint16_t));
    memcpy(_sailbot_vars.radio_buffer + sizeof(protocol_header_t) + sizeof(uint16_t), &latitude, sizeof(int32_t));
    memcpy(_sailbot_vars.radio_buffer + sizeof(protocol_header_t) + sizeof(uint16_t) + sizeof(int32_t), &longitude, sizeof(int32_t));

    size_t length = sizeof(protocol_header_t) + sizeof(uint16_t) + 2 * sizeof(int32_t);
    db_radio_disable();
    db_radio_tx(_sailbot_vars.radio_buffer, length);
}

static int8_t map_error_to_rudder_angle(float error) {
    float converted = 255.0 * error / M_PI / 2.0;

    if (converted > 127) {
        return 127;
    }
    if (converted < -128) {
        return -128;
    }
    return (int8_t)converted;
}

static float calculate_error(float heading, float bearing) {
    float difference = bearing - heading;

    if (difference < -M_PI) {
        difference += 2 * M_PI;
    }
    if (difference > M_PI) {
        difference -= 2 * M_PI;
    }
    return difference;
}

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks();
    if (ticks > _sailbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS && _sailbot_vars.ts_last_packet_received > 0) {
        if (_sailbot_vars.autonomous_operation && _sailbot_vars.radio_override) {
            _sailbot_vars.radio_override = false;
        }

        if (!_sailbot_vars.autonomous_operation) {
            // set the servos
            servos_set(0, _sailbot_vars.sail_trim);
        }
    }
}

static void _advertise(void) {
    _sailbot_vars.advertise = true;
}

static void convert_geographical_to_cartesian(cartesian_coordinate_t *out, const protocol_gps_coordinate_t *in) {
    assert(in->latitude <= (90 * 1e6) && in->latitude >= (-90 * 1e6));
    assert(in->longitude <= (180 * 1e6) && in->longitude >= (-180 * 1e6));

    // x = R*(longitude - longitude_at_origin) * cos(PHI_0)
    out->x = (((float)in->longitude / 1e6) - CONST_ORIGIN_COORD_SYSTEM_LONG) * CONST_EARTH_RADIUS_KM;
    out->x *= CONST_COS_PHI_0;
    // account for longitude in degrees by converting to radians and then to meters (from km)
    out->x *= M_PI;
    out->x = out->x * 100.0 / 18.0;  // multiply by 1000, divide by 180

    // y = R(latitude - latitude_at_origin))
    out->y = (((float)in->latitude / 1e6) - CONST_ORIGIN_COORD_SYSTEM_LAT) * CONST_EARTH_RADIUS_KM;
    // account for latitude in degrees by converting to radians and then to meters (from km)
    out->y *= M_PI;
    out->y = out->y * 100.0 / 18.0;
}

static float _distance(const cartesian_coordinate_t *pos1, const cartesian_coordinate_t *pos2) {
    return sqrtf(powf(pos2->x - pos1->x, 2) + powf(pos2->y - pos1->y, 2));  // in meters
}
