/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ping                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sends a 1 byte response back to host PC to signal a functioning        * 
*       serial connection                                                      *
*                                                                              *
*******************************************************************************/
void ping
    (
    UART_HandleTypeDef *huart /* UART Handle */
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
uint8_t response = 0x04; /* A0002 Rev 1.0 Response Code */


/*------------------------------------------------------------------------------
 Command Implementation                                                         
------------------------------------------------------------------------------*/
HAL_UART_Transmit(huart, &response, 1, 1); /* Transmit Response to USB port */


} /* ping */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
