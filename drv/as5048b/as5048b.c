/**
 * @file
 * @ingroup     drv_as5048b
 * @author      Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * @brief       Module for reading the AS5048B rotary encoder.
 *
 * @copyright   Inria, 2024
 *
 */

#include <stdio.h>
#include <stdlib.h>

// Include BSP packages
#include "gpio.h"
#include "i2c.h"
#include "as5048b.h"


//=========================== defines ==========================================
// Define scl and sda gpio pins
static const gpio_t scl = { .port = 0, .pin = 10 }; 
static const gpio_t sda = { .port = 0, .pin = 9  };

// Hardware settings of pins A1 and A2 define the last two bits of the slave address
// Set A1 and A2 to GND to get default address 0x40
#define AS5048B_SLAVE_ADDRESS   (0x40)
#define SLAVE_ADDRESS_REG       (0x15)

// Registers where absolute angle is stored
// 14 bits in total, 6-13(MSB) and 0-5(LSB) respectively
#define AS5048B_ANGLE_8MSB         (0xFE)
#define AS5048B_ANGLE_6LSB         (0xFF)

// Define Pi
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

//============================== public ========================================

// Initialise I2C communication with rotary encoder
void as5048b_init(void) {
    uint16_t    dummy_data;

    // AS5048B doesn't have a WHO_AM_I register
    db_i2c_init(&scl, &sda);

    // Trigger dummy read of data
    as5048b_i2c_read_raw_angle(&dummy_data);
}

// Reads two 8-bit registers where the 14-bit raw absolute angle is stored
void as5048b_i2c_read_raw_angle(uint16_t* angle_uint_out) {
    uint8_t data_registers[2];

    // Read registers directly via I2C
    db_i2c_begin();
    db_i2c_read_regs(AS5048B_SLAVE_ADDRESS, AS5048B_ANGLE_8MSB, &data_registers[0], 1);
    db_i2c_read_regs(AS5048B_SLAVE_ADDRESS, AS5048B_ANGLE_6LSB, &data_registers[1], 1);
    db_i2c_end();

    // Combine MSB and LSB to get the 14-bit angle value
    *angle_uint_out = (data_registers[0] << 6) | data_registers[1];
}

// Reads angle as a float in radians
void as5048b_i2c_read_angle_radian(float* angle_rad_out) {
    uint16_t    angle_uint;
    as5048b_i2c_read_raw_angle(&angle_uint);

    // Convert 14-bit raw angle (0x0 to 0x3FFF) to radians [0, 2*M_PI)
    *angle_rad_out = (float) as5048b_convert_raw_angle(angle_uint, 2*M_PI);
}

// Reads angle as a float in degrees
void as5048b_i2c_read_angle_degree(float* angle_deg_out) {
    uint16_t    angle_uint;
    as5048b_i2c_read_raw_angle(&angle_uint);

    // Convert 14-bit raw angle (0x0 to 0x3FFF) to degrees [0, 360)
    *angle_deg_out = (float) as5048b_convert_raw_angle(angle_uint, 360.);
}

// Convert the raw angle to [0, max_angle)
float as5048b_convert_raw_angle(uint16_t raw_angle, float max_angle) {
    // Magic number is to exclude max_angle
    float angle_out = ( (float)raw_angle / (float)0x3FFF ) * max_angle * (0.999938961118232313984);
    return angle_out;
}
