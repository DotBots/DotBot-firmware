/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <nrf.h>
#include "move.h"

//=========================== main =============================================

int main(void) {

    db_move_init();
    db_move_straight(500, 50);
    db_move_rotate(90, 45);
    db_move_straight(500, 50);
    db_move_rotate(90, 45);
    db_move_straight(500, 50);
    db_move_rotate(90, 45);
    db_move_straight(500, 50);

    db_move_straight(500, -50);
    db_move_rotate(90, -45);
    db_move_straight(500, -50);
    db_move_rotate(90, -45);
    db_move_straight(500, -50);
    db_move_rotate(90, -45);
    db_move_straight(500, -50);

    while (1) {
        __WFE();
    }
}
