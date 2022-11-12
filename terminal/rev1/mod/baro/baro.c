/*******************************************************************************
*
* FILE: 
* 		baro.c
*
* DESCRIPTION: 
* 		Contains API function to the engine controller
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "baro.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       verifies sensor can accessed (0x76)                                                                       *
*                                                                              *
*******************************************************************************/
uint8_t baro_get_device_id
	(
    void
    )
{

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/


/* Return the response code */
return void;

} /* ign_cmd_execute */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_pressure                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Polls each continuity pin and sets the continuity bits in the          *
*       response code                                                          *   
*                                                                              *
*******************************************************************************/
IGN_STAT ign_get_cont_info
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STAT ign_status = 0; /* Status code to be returned */


/*------------------------------------------------------------------------------
 Call API functions 
------------------------------------------------------------------------------*/

/* Poll the ematch continuity pin */
if (ematch_cont())
	{
    ign_status |= IGN_E_CONT_MASK;
    }

/* Poll the solid propellant continuity pin */
if (solid_prop_cont())
	{
    ign_status |= IGN_SP_CONT_MASK;
    }

/* Poll the nozzle continuity pin */
if (nozzle_cont())
	{
    ign_status |= IGN_NOZ_CONT_MASK;
    }

/* Return the status code */
return ign_status;
}
