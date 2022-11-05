/*******************************************************************************
*
* FILE: 
* 		sensor.h
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/

// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE    ( 0x01 )
#define SENSOR_POLL_CODE    ( 0x02 )

/* General */
#define NUM_SENSORS         ( 1   )
#define IMU_DATA_SIZE       ( 18  )
#define SENSOR_DATA_SIZE	( 26  )

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
typedef enum 
	{
    SENSOR_OK = 0         ,
	SENSOR_UNRECOGNIZED_OP,
	SENSOR_UNSUPPORTED_OP ,
	SENSOR_IMU_FAIL       ,
    SENSOR_FAIL
    } SENSOR_STATUS;

typedef SENSOR_DATA
	{
	uint32_t timer,
	IMU_DATA imu_data,
	// TODO: Pass the real baro values
	uint16_t dummy_baro_pressure,
	uint16_t dummy_baro_temp	
	}

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/* Execute a sensor subcommand */
uint8_t sensor_cmd_execute
	(
	uint8_t subcommand
    );

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    uint8_t* pSensor_buffer 
    );


#endif /* IMU_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
