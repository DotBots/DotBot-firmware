/**
 * @file
 * @ingroup bsp_board
 *
 * @brief  nRF52833-specific definition of the "board" bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <nrf.h>

#include "board.h"
#include "board_config.h"
#include "gpio.h"

//=========================== defines =========================================

//=========================== variables =========================================

#if defined(DB_REGULATOR_PORT)
static const gpio_t _reg_pin = { .port = DB_REGULATOR_PORT, .pin = DB_REGULATOR_PIN };
#endif

#if defined(DB_RELAY_SW_PORT)
static const gpio_t _relay_ws_pin = { .port = DB_RELAY_SW_PORT, .pin = DB_RELAY_SW_PIN };
#endif

//=========================== public ==========================================

void db_board_init(void) {

#if defined(BOARD_LH2_MINI_MOTE)
    // Make sure the mini-mote is running at 3.0v
    // Might need a re-start to take effect
    if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V0) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
        }
        NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V0;

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
        }
    }

    // Start the mini-mote with the RGB LED turned off
    static const gpio_t _r_led_pin = { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN };
    static const gpio_t _g_led_pin = { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN };
    static const gpio_t _b_led_pin = { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN };

    // Configure RGB LED pins as output
    db_gpio_init(&_r_led_pin, DB_GPIO_OUT);
    db_gpio_init(&_g_led_pin, DB_GPIO_OUT);
    db_gpio_init(&_b_led_pin, DB_GPIO_OUT);
    // Shutdown the LED
    db_gpio_set(&_r_led_pin);
    db_gpio_set(&_g_led_pin);
    db_gpio_set(&_b_led_pin);

#endif

#if defined(DB_RELAY_SW_PORT)
    db_gpio_init(&_relay_ws_pin, DB_GPIO_OUT);
    db_gpio_set(&_relay_ws_pin);
#endif

#if defined(DB_REGULATOR_PORT)
    // Turn ON the DotBot board regulator if provided
    db_gpio_init(&_reg_pin, DB_GPIO_OUT);
#if defined(BOARD_DOTBOT_V3)
    // Regulator pin correspond to motor_en and is active low
    db_gpio_clear(&_reg_pin);
#else
    db_gpio_set(&_reg_pin);
#endif
#endif
}

// TODO: add a 3v minimote UICR code
