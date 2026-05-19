/*
 * cmake_hello — minimal LED blinker validating the DotBot-firmware
 * CMake build path against the dotbot_libs INTERFACE target.
 *
 * Touches both kinds of headers we plumbed in:
 *   * dotbot-libs (gpio.h, board.h, timer.h, board_config.h) — via
 *     the `dotbot_libs` INTERFACE target's PUBLIC include dirs.
 *   * Nordic MDK / ARM CMSIS-Core (nrf.h → __WFE intrinsic) — via
 *     the FetchContent paths exposed as ${NRFX_MDK_INCLUDE} and
 *     ${CMSIS_CORE_INCLUDE} from the root CMakeLists.
 *
 * Pin selection comes from the BOARD_* preprocessor define
 * (BOARD_DOTBOT_V3 by default). DB_LED1 → P1.05 on DotBot v3 —
 * the blue channel of the discrete RGB LED.
 *
 * Blink is interrupt-driven: a 500 ms periodic timer toggles the
 * LED in its ISR; main() just sleeps on WFE between events.
 */

#include "nrf.h"            /* __WFE — confirms CMSIS-Core is on the path */
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "timer.h"

#define TIMER_DEV  0
#define BLINK_MS   500

static const gpio_t _heartbeat = { .port = DB_LED1_PORT, .pin = DB_LED1_PIN };

static void on_tick(void) {
    db_gpio_toggle(&_heartbeat);
}

int main(void) {
    db_board_init();
    db_gpio_init(&_heartbeat, DB_GPIO_OUT);
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, BLINK_MS, on_tick);

    for (;;) {
        __WFE();   /* sleep until the timer ISR (or any event) fires */
    }
}
