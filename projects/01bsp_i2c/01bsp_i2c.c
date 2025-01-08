/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the I2C api.
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdio.h>
#include "board.h"
#include "board_config.h"
#include "i2c.h"
#include "timer_hf.h"

//=========================== defines ==========================================

#define LSM303AGR_ADDR         (0x19)
#define LSM303AGR_WHO_AM_I_VAL (0x33)

#define LSM303AGR_WHO_AM_I_REG (0x0F)
#define LSM303AGR_CTRL1_A_REG  (0x20)
#define LSM303AGR_OUT_X_L_A    (0x28)
#define LSM303AGR_OUT_X_H_A    (0x29)
#define LSM303AGR_OUT_Y_L_A    (0x2A)
#define LSM303AGR_OUT_Y_H_A    (0x2B)
#define LSM303AGR_OUT_Z_L_A    (0x2C)
#define LSM303AGR_OUT_Z_H_A    (0x2D)

#define I2C_DEV (0)

typedef struct {
    int16_t x;  ///< X axis
    int16_t y;  ///< Y axis
    int16_t z;  ///< Z axis
} lsm303agr_acc_data_t;

//=========================== main =============================================

int main(void) {
    db_board_init();
    db_timer_hf_init(0);
    db_i2c_init(I2C_DEV, &db_scl, &db_sda);
    db_i2c_begin(I2C_DEV);
    uint8_t who_am_i;
    db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_WHO_AM_I_REG, &who_am_i, 1);
    if (who_am_i != LSM303AGR_WHO_AM_I_VAL) {
        printf(
            "Invalid WHO_AM_I value '%d' (expected: %d)\n",
            who_am_i, LSM303AGR_WHO_AM_I_VAL);
        return -1;
    }

    // enable the accelerometer
    uint8_t reg = 0x57;
    db_i2c_write_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_CTRL1_A_REG, &reg, 1);
    db_i2c_end(I2C_DEV);

    lsm303agr_acc_data_t data;
    uint8_t              tmp = 0;

    // read accelerometer data in a loop
    while (1) {
        db_i2c_begin(I2C_DEV);
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_X_L_A, &tmp, 1);
        data.x = tmp;
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_X_H_A, &tmp, 1);
        data.x |= tmp << 8;
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_Y_L_A, &tmp, 1);
        data.y = tmp;
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_Y_H_A, &tmp, 1);
        data.y |= tmp << 8;
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_Z_L_A, &tmp, 1);
        data.z = tmp;
        db_i2c_read_regs(I2C_DEV, LSM303AGR_ADDR, LSM303AGR_OUT_Z_H_A, &tmp, 1);
        data.z |= tmp << 8;
        db_i2c_end(I2C_DEV);
        printf("x: %i, y: %i, z: %i\n", data.x, data.y, data.z);
        db_timer_hf_delay_ms(0, 200);
    }
}
