/**
 * @file 03app_sailbot.c
 * @author Mališa Vučinić <malisa.vucinic@inria.fr>
 * @brief This is the radio-controlled SailBot app.
 *
 * Load this program on your board. Now the SailBot can be remotely controlled
 * from a nearby nRF52840-DK.
 *
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>

// Include BSP packages
#include "device.h"
#include "radio.h"
#include "servos.h"
#include "gps.h"
#include "lis2mdl.h"
#include "protocol.h"
#include "timer.h"
#include "timer_hf.h"
#include "gpio.h"
#include "assert.h"
#include "math.h"

//=========================== defines =========================================

#define AUTONOMOUS_OPERATION (0)  //< user define to enable autonomous operation

#define PATH_PLANNER_PERIOD_MS (500)  //< path planner period
#define CONTROL_LOOP_PERIOD_MS (100)  //< control loop period

#define SAIL_TRIM_ANGLE_UNIT_STEP (10)     //< unit step increase/decrease when trimming the sails
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define TIMEOUT_CHECK_DELAY_MS    (200)    ///< 200 ms delay between packet received timeout checks
#define ADVERTISEMENT_PERIOD_MS   (500)    ///< send an advertisement every 500 ms
#define DB_BUFFER_MAX_BYTES       (64U)    ///< Max bytes in UART receive buffer
#define MAX_WAYPOINTS             (10)

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
#define CONST_ORIGIN_COORD_SYSTEM_LAT  CONST_METRO_BERCY_LAT
#define CONST_ORIGIN_COORD_SYSTEM_LONG CONST_METRO_BERCY_LONG
#define CONST_COS_PHI_0                CONST_COS_PHI_0_INRIA_PARIS

typedef struct {
    uint8_t valid;
    float   latitude;
    float   longitude;
} waypoint_t;

typedef struct {
    float x;
    float y;
} cartesian_coordinate_t;

typedef struct {
    uint32_t   ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    int8_t     sail_trim;                          ///< Last angle of the servo controlling sail trim
    waypoint_t waypoints[MAX_WAYPOINTS];           ///< Array containing pre-programmed waypoints
    waypoint_t next_waypoint;                      ///< The next waypoint SailBot is going to traverse
    uint8_t    radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    bool       autonomous_operation;               ///< Flag used to enable/disable autonomous operation
    bool       radio_override;                     ///< Flag used to override autonomous operation when radio-controlled
} sailbot_vars_t;

//=========================== variables =========================================

static const gpio_t   _led1_pin     = { .pin = 15, .port = 0 };
static sailbot_vars_t _sailbot_vars = { 0, 0, {
                                                  // waypoints
                                                  { 1, 48.84226853349436, 2.38764801698452 },  // waypoint 1 jardin de reuilly
                                                  { 0, 48.85292599987966, 2.3691622320143892 },    // waypoint 2 bastille
                                                  { 0, 48.84411394, 2.318446164 },  // waypoint 3
                                                  { 0, 0, 0 },                      // waypoint 4
                                                  { 0, 0, 0 },                      // waypoint 5
                                                  { 0, 0, 0 },                      // waypoint 6
                                                  { 0, 0, 0 },                      // waypoint 7
                                                  { 0, 0, 0 },                      // waypoint 8
                                                  { 0, 0, 0 },                      // waypoint 9
                                                  { 0, 0, 0 },                      // waypoint 10
                                              } };

//=========================== prototypes =========================================

void          radio_callback(uint8_t *packet, uint8_t length);
void          path_planner_callback(void);
void          control_loop_callback(void);
static void   convert_geographical_to_cartesian(cartesian_coordinate_t *out, waypoint_t *in);
static float  calculate_error(float heading, float bearing);
static int8_t map_error_to_rudder_angle(float error);
static void   _timeout_check(void);
static void   _advertise(void);

//=========================== main =========================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    cartesian_coordinate_t temp;

    // Turn ON the LED1
    NRF_P0->DIRSET = 1 << _led1_pin.pin;  // set pin as output
    NRF_P0->OUTSET = 0 << _led1_pin.pin;  // set pin LOW

    _sailbot_vars.autonomous_operation = AUTONOMOUS_OPERATION;
    _sailbot_vars.radio_override       = false;
    _sailbot_vars.sail_trim            = 50;

    // Configure Radio as a receiver
    db_radio_init(&radio_callback);  // Set the callback function.
    db_radio_set_frequency(8);       // Set the RX frequency to 2408 MHz.
    db_radio_rx_enable();            // Start receiving packets.

    // Init the IMU
    lis2mdl_init(NULL);

    // Configure Motors
    servos_init();
    servos_set(0, _sailbot_vars.sail_trim);

    // init the timers
    db_timer_init();
    db_timer_hf_init();

    // Configure GPS without callback
    gps_init(NULL);

    // set timer callbacks
    db_timer_set_periodic_ms(0, TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(1, ADVERTISEMENT_PERIOD_MS, &_advertise);

    if (_sailbot_vars.autonomous_operation) {
        db_timer_set_periodic_ms(2, PATH_PLANNER_PERIOD_MS, &path_planner_callback);
        db_timer_hf_set_periodic_us(0, CONTROL_LOOP_PERIOD_MS * 1000, &control_loop_callback);

        // convenience dump of the pre-programmed waypoints and their conversion
        for (uint8_t i = 0; i < MAX_WAYPOINTS; i++) {
            if (_sailbot_vars.waypoints[i].valid) {
                printf("Pre-programmed waypoint %d: %f %f; ", i, _sailbot_vars.waypoints[i].latitude, _sailbot_vars.waypoints[i].longitude);
                convert_geographical_to_cartesian(&temp, &_sailbot_vars.waypoints[i]);
                printf("converted: %f %f\n", temp.x, temp.y);
            }
        }
    }

    // Wait for radio packets to arrive
    while (1) {
        // processor idle until an interrupt occurs and is handled
        // if IMU data is ready, trigger the I2C transfer from outside the interrupt context
        if (lis2mdl_data_ready()) {
            lis2mdl_read_heading();
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
    uint8_t *          ptk_ptr = packet;
    protocol_header_t *header  = (protocol_header_t *)ptk_ptr;

    // timestamp the arrival of the packet
    _sailbot_vars.ts_last_packet_received = db_timer_ticks();
    _sailbot_vars.radio_override = true;

    // we filter out all packets other than MOVE_RAW command
    if (header->type != DB_PROTOCOL_CMD_MOVE_RAW) {
        return;
    }

    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != db_device_id()) {
        return;
    }

    // Check version is compatible
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Only move raw commands are supported
    if (header->type != DB_PROTOCOL_CMD_MOVE_RAW) {
        return;
    }

    // Read the DotBot command
    protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)(ptk_ptr + sizeof(protocol_header_t));

    if (command->right_y > 0 && ((int16_t)_sailbot_vars.sail_trim + SAIL_TRIM_ANGLE_UNIT_STEP < 127)) {
        _sailbot_vars.sail_trim += SAIL_TRIM_ANGLE_UNIT_STEP;
    } else if (command->right_y < 0 && ((int16_t)_sailbot_vars.sail_trim - SAIL_TRIM_ANGLE_UNIT_STEP > 0)) {
        _sailbot_vars.sail_trim -= SAIL_TRIM_ANGLE_UNIT_STEP;
    }

    // set the servos
    servos_set(command->left_x, _sailbot_vars.sail_trim);
}

// set _sailbot_vars.next_waypoint
void path_planner_callback(void) {
    uint8_t i;

    // TODO
    // check if we have reached the current waypoint before updating the next one
    // if reached, invalidate the current waypoint

    // TODO
    // path plan the sailing route given wind conditions

    // update the next_waypoint
    for (i = 0; i < MAX_WAYPOINTS; i++) {
        if (_sailbot_vars.waypoints[i].valid) {
            // copy the first found valid waypoint into the next_waypoint
            memcpy(&_sailbot_vars.next_waypoint, &_sailbot_vars.waypoints[i], sizeof(waypoint_t));
        }
    }
}

void control_loop_callback(void) {
    nmea_gprmc_t *         gps_data;
    waypoint_t             current_position_gps;
    cartesian_coordinate_t target;
    cartesian_coordinate_t position;
    float                  theta;
    float                  psi;
    float                  error;
    float                  heading;
    int8_t                 rudder_angle;

    gps_data = gps_last_known_position();
    heading  = lis2mdl_last_heading();
    if (gps_data->valid && _sailbot_vars.next_waypoint.valid) {
        // convert the next_waypoint to local coordinate system (and copy to stack to avoid concurrency issues)
        convert_geographical_to_cartesian(&target, &_sailbot_vars.next_waypoint);

        // save the current GPS position on stack to avoid concurrency issues between control_loop_callback() and the GPS module
        current_position_gps.latitude  = gps_data->latitude;
        current_position_gps.longitude = gps_data->longitude;
        current_position_gps.valid     = 1;

        // convert geographical data given by GPS to our local coordinate system
        convert_geographical_to_cartesian(&position, &current_position_gps);

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
}

static int8_t map_error_to_rudder_angle(float error) {
    float converted;

    converted = 255.0 * error / M_PI / 2.0;

    if (converted > 127) {
        return 127;
    }
    if (converted < -128) {
        return -128;
    }
    return (int8_t)converted;
}

static float calculate_error(float heading, float bearing) {
    float difference;

    difference = bearing - heading;

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
        _sailbot_vars.radio_override = false;
        // set the servos
        servos_set(0, _sailbot_vars.sail_trim);
    }
}

static void _advertise(void) {
    db_protocol_header_to_buffer(_sailbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DB_PROTOCOL_ADVERTISEMENT);
    size_t length = sizeof(protocol_header_t);
    db_radio_rx_disable();
    db_radio_tx(_sailbot_vars.radio_buffer, length);
    db_radio_rx_enable();
}

static void convert_geographical_to_cartesian(cartesian_coordinate_t *out, waypoint_t *in) {
    assert(in->valid > 0);
    assert(in->latitude <= 90.0 && in->latitude >= -90.0);
    assert(in->longitude <= 180.0 && in->longitude >= -180);

    // x = R*(longitude - longitude_at_origin) * cos(PHI_0)
    out->x = (in->longitude - CONST_ORIGIN_COORD_SYSTEM_LONG) * CONST_EARTH_RADIUS_KM;
    out->x *= CONST_COS_PHI_0;
    // account for longitude in degrees by converting to radians and then to meters (from km)
    out->x *= M_PI;
    out->x = out->x * 100.0 / 18.0;  // multiply by 1000, divide by 180

    // y = R(latitude - latitude_at_origin))
    out->y = (in->latitude - CONST_ORIGIN_COORD_SYSTEM_LAT) * CONST_EARTH_RADIUS_KM;
    // account for latitude in degrees by converting to radians and then to meters (from km)
    out->y *= M_PI;
    out->y = out->y * 100.0 / 18.0;
}
