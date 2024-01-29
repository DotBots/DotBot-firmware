/**
 * @file
 * @defgroup project_Radio    Radio application
 * @ingroup projects
 * @brief This is the radio tx code for radio test
 
 *
 * @author Raphael Simoes <raphael.simoes@inria.fr>
 * @copyright Inria, 2024
 */

#include <nrf.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
// Include BSP headers
#include "uart.h"
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "protocol.h"
#include "radio.h"
#include "timer.h"


#define DB_BUFFER_MAX_BYTES (255U)       ///< Max bytes in UART receive buffer



static void _led1_blink_fast(void) {

      db_gpio_toggle(&db_led1);

}

static void _radio_callback(uint8_t *packet, uint8_t length) {
    uint8_t  *data = packet;
    uint8_t l = length;
    if (*data< l){};
    return;

}


int main(void)
{
    db_gpio_init(&db_led1, DB_GPIO_OUT);  // Global status
    db_gpio_set(&db_led1);
    db_timer_init();
    db_timer_set_periodic_ms(0, 1000, _led1_blink_fast);  

    db_protocol_init();
    db_radio_init(&_radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx();              // Start receiving packets.
    
    uint8_t radio_tx_buffer[DB_BUFFER_MAX_BYTES]; 
    uint8_t packet_counter = 0;

    while(1){

            //protocol_move_raw_command_t command = { 'T' , 'E' , 'S' , 'T' };  
            //db_protocol_cmd_move_raw_to_buffer(radio_tx_buffer, DB_BROADCAST_ADDRESS, DotBot, *command);
            
            db_uint8_t_to_buffer(radio_tx_buffer, DB_BROADCAST_ADDRESS, DotBot, &packet_counter);

            db_radio_disable();
            //db_radio_tx(radio_tx_buffer, sizeof(protocol_header_t) + sizeof(protocol_move_raw_command_t));
            db_radio_tx(radio_tx_buffer, sizeof(protocol_header_t) + sizeof(int8_t));
            packet_counter++;

            db_timer_delay_ms(500);
    }
   
}