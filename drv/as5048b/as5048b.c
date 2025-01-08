/**
 * @file
 * @ingroup     drv_as5048b
 * @author      Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * @brief       Module for reading the AS5048B rotary encoder
 *
 * @copyright   Inria, 2024
 *
 */

#include <stdint.h>
#include <math.h>

// Include BSP packages
#include "board_config.h"
#include "i2c.h"
#include "as5048b.h"

//=========================== defines ==========================================
// Define scl and sda gpio pins
// db_scl and db_sda are defined in board_config.h
// For SailBot PCB: sda=TP3, scl=TP4

// Hardware settings of pins A1 and A2 define the last two bits of the slave address
// Set A1 and A2 to GND to get default address 0x40
#define AS5048B_SLAVE_ADDRESS (0x40)
#define SLAVE_ADDRESS_REG     (0x15)

// Registers where absolute angle is stored
// 14 bits in total, 6-13(MSB) and 0-5(LSB) respectively
#define AS5048B_ANGLE_8MSB (0xFE)
#define AS5048B_ANGLE_6LSB (0xFF)

#define I2C_DEV (0)

//============================== public ========================================

// Initialise I2C communication with rotary encoder
void as5048b_init(void) {
    // AS5048B doesn't have a WHO_AM_I register
    db_i2c_init(I2C_DEV, &db_scl, &db_sda);
}

// Reads two 8-bit registers where the 14-bit raw absolute angle is stored
uint16_t as5048b_i2c_read_raw_angle(void) {
    uint8_t data_registers[2];

    // Read registers directly via I2C
    db_i2c_begin(I2C_DEV);
    db_i2c_read_regs(I2C_DEV, AS5048B_SLAVE_ADDRESS, AS5048B_ANGLE_8MSB, &data_registers[0], 1);
    db_i2c_read_regs(I2C_DEV, AS5048B_SLAVE_ADDRESS, AS5048B_ANGLE_6LSB, &data_registers[1], 1);
    db_i2c_end(I2C_DEV);

    // Combine MSB and LSB to get the 14-bit angle value
    return (uint16_t)(data_registers[0] << 6) | data_registers[1];
}

// Read angle as a float in radians
float as5048b_i2c_read_angle_radian(void) {
    uint16_t angle_uint = as5048b_i2c_read_raw_angle();

    // Convert 14-bit raw angle (0x0 to 0x3FFF) to radians [0, 2*M_PI)
    return (float)as5048b_convert_raw_angle(angle_uint, 2 * M_PI);
}

// Read angle as a float in degrees
float as5048b_i2c_read_angle_degree(void) {
    uint16_t angle_uint = as5048b_i2c_read_raw_angle();

    // Convert 14-bit raw angle (0x0 to 0x3FFF) to degrees [0, 360)
    return (float)as5048b_convert_raw_angle(angle_uint, 360.);
}

// Convert the raw angle to [0, max_angle)
float as5048b_convert_raw_angle(uint16_t raw_angle, float max_angle) {
    // Magic number is there to exclude max_angle
    float angle_out = ((float)raw_angle / (float)0x3FFF) * max_angle * (0.999938961118232313984);
    return angle_out;
}
