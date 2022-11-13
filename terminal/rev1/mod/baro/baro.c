/*******************************************************************************
*
* FILE: 
* 		baro.c
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
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
#include "sdr_pin_defines_A0002_rev1.h"
#include "baro.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1; /* MCU I2C handle */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		BARO_Read_Register                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read one register for BARO 											   *
*                                                                              *
*******************************************************************************/
BARO_STATUS BARO_Read_Register
    (
    uint8_t reg_addr, 
    uint8_t *pData
    )
{

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c1, 
                            BARO_I2C_ADDR, 
                            reg_addr, 	
                            I2C_MEMADD_SIZE_8BIT, 
                            pData, 
                            sizeof( uint8_t ), 
                            HAL_DEFAULT_TIMEOUT 
                            ); 

if (hal_status != HAL_TIMEOUT){
return BARO_OK;
}
else
{
return BARO_TIMEOUT;
}
} /* BARO_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		BARO_Read_Registers                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time for BARO            *
*                                                                              *
*******************************************************************************/
BARO_STATUS BARO_Read_Registers
    (
    uint8_t reg_addr, 
    uint8_t *pData, 
    uint8_t num_registers
    )
{

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c1, 
                            BARO_I2C_ADDR, 
                            reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, 
                            pData, 
                            num_registers, 
                            HAL_DEFAULT_TIMEOUT
                            );

if (hal_status != HAL_TIMEOUT)
{
return BARO_OK;
}
else
{
return BARO_TIMEOUT;
}
} /* BARO_Read_Registers */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		BARO_Write_Register                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Write one register to the BARO                                         *
*                                                                              *
*******************************************************************************/

BARO_STATUS BARO_Write_Register
    (
    uint8_t reg_addr, 
    uint8_t *pData
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write
							(
							&hi2c1, 
							BARO_I2C_ADDR, 
							reg_addr, 
							I2C_MEMADD_SIZE_8BIT, 
							pData, 
							sizeof( uint8_t ), 
							HAL_DEFAULT_TIMEOUT
							);

if (hal_status != HAL_TIMEOUT)
{
return BARO_OK;
}
else
{
return BARO_TIMEOUT;
}
} /* BARO_Write_Register */



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Gets the device ID of the barometric pressure sensor, primarily used   *
*       to verify that the sensor can be accessed by the MCU                   *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id_ptr /* reference to memory where id is returned */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 API Function implementation 
------------------------------------------------------------------------------*/

/* Read baro register with I2C */
hal_status = HAL_I2C_Mem_Read (
                               &hi2c1              ,
                               BARO_I2C_ADDR       ,
                               BARO_REG_CHIP_ID    ,
                               I2C_MEMADD_SIZE_8BIT,
							   baro_id_ptr         ,
							   sizeof( uint8_t )   ,
                               HAL_DEFAULT_TIMEOUT
                              );

/* Check HAL Status and return data if okay */
switch ( hal_status )
	{
	case HAL_OK: 
		return BARO_OK;
		break;

	case HAL_TIMEOUT:
		return BARO_TIMEOUT;
		break;

	case HAL_ERROR:
		return BARO_I2C_ERROR;
		break;

	default:
		return BARO_UNRECOGNIZED_HAL_STATUS;
		break;
	}

} /* baro_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_pressure                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a pressure reading from the sensor                           *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_pressure
	(
    uint32_t *baro_pressure_data_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t 	regPres[3];
BARO_STATUS baro_status;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive pressure data registers */  
baro_status = BARO_Read_Registers
								(
								BARO_PRESSURE_DATA,
								&regPres[0], 
								3
								);

/* Check for HAL BARO error */
if ( baro_status == BARO_TIMEOUT )
{
	return BARO_TIMEOUT;
}

/* Combine all bytes value to 24 bit value */
uint32_t baro_pressure_raw = ( ( (uint32_t) regPres[2] << 16 ) |
							   ( (uint32_t) regPres[1] << 8 )  | 
							   ( (uint32_t) regPres[0] ) );

/* Export data */
*baro_pressure_data_ptr = baro_pressure_raw;

return BARO_OK;
} /* baro_get_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_temp                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a temperature reading from the sensor                        *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_temp
	(
    uint32_t* baro_temp_data_ptr
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t 	regPres[3];
BARO_STATUS baro_status;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read 3 consecutive temperature data registers */  
baro_status = BARO_Read_Registers
								(
								BARO_TEMPERATURE_DATA,
								&regPres[0], 
								3
								);

/* Check for HAL BARO error */
if ( baro_status == BARO_TIMEOUT )	
{
	return BARO_TIMEOUT;
}

/* Combine all bytes value to 24 bit value */
uint32_t baro_temp_raw = ( ( (uint32_t) regPres[2] << 16 ) | 
						   ( (uint32_t) regPres[1] << 8 ) | 
						   ( (uint32_t) regPres[0] ) );

/* Export data */
*baro_temp_data_ptr = baro_temp_raw;


return BARO_OK;
} /* baro_get_temp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_altitude                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		gets the altitude of the rocket from the sensor readouts               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_altitude
	(
    void
	)
{
return BARO_OK;
} /* baro_get_altitude */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
