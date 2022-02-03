/**
 * @file 01bsp_radio_txrx.c
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the onboard Radio in the DotBot board.
 * 
 * This program is the RX part.
 * if the corresponding compatible TX program is loaded into a nearby nRF52840-DK,
 * pressing the buttons in the DK should toggle ON and OFF the P0.31 pin on the DotBot (P0.31).
 * 
 * 
 * @copyright Inria, 2022
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== defines =========================================

//=========================== variables =========================================

//=========================== main =========================================

/**
*  @brief The program starts executing here.
*/
int main(void)
{

 

  while (1)
  {

  }

  // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
  __NOP();
}
