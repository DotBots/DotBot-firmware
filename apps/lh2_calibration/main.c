/**
 * @file
 * @date 2022
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 *
 */

#include <string.h>
#include <nrf.h>

#include "board.h"
#include "board_config.h"
#include "hdlc.h"
#include "lh2.h"
#include "timer.h"
#include "uart.h"

#define DB_BUFFER_MAX_BYTES    (128U)     ///< max bytes in data buffer
#define DB_UART_INDEX          (0U)       ///< UART index
#define DB_UART_BAUDRATE       (115200U)  ///< UART baudrate
#define DB_LH2_UPDATE_DELAY_MS (500U)     ///< 100ms delay between each LH2 data refresh
#define TIMER_DEV              (1)

typedef struct {
    uint8_t  data_buffer[DB_BUFFER_MAX_BYTES];      ///< Internal buffer used for storing raw calibration data
    uint8_t  hdlc_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Internal buffer used for sending serial HDLC frames
    db_lh2_t lh2;
    bool     update_lh2;
} calibration_vars_t;

static calibration_vars_t _app_vars;

static void _update_lh2(void) {
    _app_vars.update_lh2 = true;
}

int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    db_lh2_init(&_app_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    // Initialize the main UART, only used for sending data
    db_uart_init(DB_UART_INDEX, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, NULL);

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_LH2_UPDATE_DELAY_MS, &_update_lh2);

    while (1) {
        __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_app_vars.lh2);

        if (_app_vars.update_lh2) {
            db_lh2_stop();
            for (uint8_t lh_index = 0; lh_index < LH2_BASESTATION_COUNT; lh_index++) {
                if (_app_vars.lh2.data_ready[0][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE && _app_vars.lh2.data_ready[1][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE) {
                    size_t length                   = 0;
                    _app_vars.data_buffer[length++] = lh_index;
                    for (uint8_t lh2_sweep_index = 0; lh2_sweep_index < LH2_SWEEP_COUNT; lh2_sweep_index++) {
                        memcpy(&_app_vars.data_buffer[length], &_app_vars.lh2.locations[lh2_sweep_index][lh_index].lfsr_counts, sizeof(uint32_t));
                        length += sizeof(uint32_t);
                        _app_vars.lh2.data_ready[lh2_sweep_index][lh_index] = DB_LH2_NO_NEW_DATA;
                    }

                    // Send the data over UART using HDLC framing
                    size_t frame_len = db_hdlc_encode(_app_vars.data_buffer, length, _app_vars.hdlc_buffer);
                    db_uart_write(DB_UART_INDEX, _app_vars.hdlc_buffer, frame_len);
                }
            }
            db_lh2_start();
            _app_vars.update_lh2 = false;
        }
    }
}
