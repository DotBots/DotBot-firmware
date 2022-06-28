/**
 * @file i2c.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "i2c" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <nrf.h>
#include "i2c.h"

//=========================== defines ==========================================

#define DB_TWIM             (NRF_TWIM1)
#define DB_TWIM_SCL_PIN     (8)
#define DB_TWIM_SCL_PORT    (0)
#define DB_TWIM_SDA_PIN     (16)
#define DB_TWIM_SDA_PORT    (0)
#define DB_TWIM_FREQ        (TWIM_FREQUENCY_FREQUENCY_K400)

//=========================== prototypes =======================================

void _wait(void);

static uint8_t tx_buf[256];
static bool tx_running = false;

//=========================== public ===========================================

/**
 * @brief Initialize the I2C peripheral.
 */
void db_i2c_init(void) {
    // Clear pending errors
    DB_TWIM->EVENTS_ERROR = 0;
    DB_TWIM->ERRORSRC = 0;
    // Ensure TWIM is disabled while configuring it
    DB_TWIM->ENABLE = TWIM_ENABLE_ENABLE_Disabled;

    // Configure TWIM pins
    NRF_P0->PIN_CNF[DB_TWIM_SCL_PIN] |= (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos) |
                                        (GPIO_PIN_CNF_DRIVE_S0D1    << GPIO_PIN_CNF_DRIVE_Pos);
    NRF_P0->PIN_CNF[DB_TWIM_SDA_PIN] |= (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos) |
                                        (GPIO_PIN_CNF_DRIVE_S0D1    << GPIO_PIN_CNF_DRIVE_Pos);

    // Configure TWIM
    DB_TWIM->PSEL.SCL   =   (DB_TWIM_SCL_PORT                   << TWIM_PSEL_SCL_PORT_Pos)  |
                            (DB_TWIM_SCL_PIN                    << TWIM_PSEL_SCL_PIN_Pos)   |
                            (TWIM_PSEL_SCL_CONNECT_Connected    << TWIM_PSEL_SCL_CONNECT_Pos);
    DB_TWIM->PSEL.SDA   =   (DB_TWIM_SDA_PORT                   << TWIM_PSEL_SDA_PORT_Pos)  |
                            (DB_TWIM_SDA_PIN                    << TWIM_PSEL_SDA_PIN_Pos)   |
                            (TWIM_PSEL_SDA_CONNECT_Connected    << TWIM_PSEL_SDA_CONNECT_Pos);

    // Set frequency
    DB_TWIM->FREQUENCY  =   DB_TWIM_FREQ;

    NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
    NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);

    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

/**
 * @brief Begin transmission on I2C.
 */
void db_i2c_begin(void) {
    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

/**
 * @brief End transmission on I2C.
 */
void db_i2c_end(void) {
    DB_TWIM->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
}

/**
 * @brief Read bytes from register.
 */
void db_i2c_read_regs(uint8_t addr, uint8_t reg, void *data, size_t len) {
    DB_TWIM->ADDRESS        = addr;
    DB_TWIM->TXD.MAXCNT     = 1;
    DB_TWIM->TXD.PTR        = (uint32_t)&reg;
    DB_TWIM->RXD.PTR        = (uint32_t)data;
    DB_TWIM->RXD.MAXCNT     = (uint8_t)len;
    DB_TWIM->SHORTS         = (1 << TWIM_SHORTS_LASTTX_STARTRX_Pos) | (1 << TWIM_SHORTS_LASTRX_STOP_Pos);
    DB_TWIM->TASKS_STARTTX  = 1;
    _wait();
}

/**
 * @brief Write bytes to register.
 */
void db_i2c_write_regs(uint8_t addr, uint8_t reg, const void *data, size_t len) {
    tx_buf[0] = reg;
    memcpy(&tx_buf[1], data, len);

    // Send the content to write to the register
    DB_TWIM->ADDRESS        = addr;
    DB_TWIM->TXD.PTR        = (uint32_t)tx_buf;
    DB_TWIM->TXD.MAXCNT     = (uint8_t)len + 1;
    DB_TWIM->SHORTS         = (1 << TWIM_SHORTS_LASTTX_STOP_Pos);
    DB_TWIM->TASKS_STARTTX  = 1;
    _wait();
}

//=========================== defines ==========================================

void _wait(void) {
    DB_TWIM->INTENSET = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
    tx_running = true;
    while (tx_running) {
        __WFE();
    }
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void) {
    if (DB_TWIM->EVENTS_STOPPED) {
        DB_TWIM->EVENTS_STOPPED = 0;
        tx_running = false;
    }

    if (DB_TWIM->EVENTS_ERROR) {
        DB_TWIM->EVENTS_ERROR = 0;
        tx_running = false;
        if (DB_TWIM->ERRORSRC & TWIM_ERRORSRC_ANACK_Msk) {
            DB_TWIM->ERRORSRC = TWIM_ERRORSRC_ANACK_Msk;
            puts("NACK on address byte");
        }
        if (DB_TWIM->ERRORSRC & TWIM_ERRORSRC_DNACK_Msk) {
            DB_TWIM->ERRORSRC = TWIM_ERRORSRC_DNACK_Msk;
            puts("NACK on data byte");
        }
    }
    DB_TWIM->INTENCLR = TWIM_INTEN_STOPPED_Msk | TWIM_INTEN_ERROR_Msk;
}
