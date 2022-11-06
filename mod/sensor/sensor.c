/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"
#include "sensor.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1; /* USB UART handler struct        */


/*------------------------------------------------------------------------------
 Public procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
    uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_subcmd_status;           /* Status indicating if 
                                                 subcommand function returned 
                                                 properly                     */
// uint32_t      sensor_readings[ NUM_SENSORS ]; /* Readings obtained from each 
//                                                  sensor                       */
// uint8_t       sensor_readings_bytes[ 4*NUM_SENSORS ];
// const uint8_t num_sensor_bytes = 4*NUM_SENSORS; /* Number of bytes to be transmitted back to PC     */

// IMU data size should be in header
// imu data, imu data size, imu data byte list, memcpy the addr of firts imu data byte and imu data, imu data size

// uint8_t      sensor_data_byte[ IMU_DATA_SIZE ];    /*Initialize sensor data byte array
//                                                      with size of all sensor data*/
// const uint8_t num_sensor_bytes = IMU_DATA_SIZE;

// TODO: Implement sensor_data_byte with a size of sensor data
uint8_t      sensor_data_byte[ SENSOR_DATA_SIZE ]; /*Initialize sensor data byte array
                                                     with size of all sensor data*/
const uint8_t num_sensor_bytes = SENSOR_DATA_SIZE;

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/

// /* Set sensor readings to zero */
// for ( uint8_t i = 0; i < NUM_SENSORS; ++i )
// 	{
// 	sensor_readings[i] = 0;
//     }
// for ( uint8_t i = 0; i < num_sensor_bytes; ++i )
// 	{
//     sensor_readings_bytes[i] = 0;
//     }

/* Set sensor readings to zero */
memset
( 
&sensor_data_byte[0],
0,
sizeof( sensor_data_byte )
);


/*------------------------------------------------------------------------------
 Execute Sensor Subcommand 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{

	/* Poll Sensors continuously */
    case SENSOR_POLL_CODE:
		{
		// TODO: Implement sensor poll function 
		return ( SENSOR_UNSUPPORTED_OP );
        } /* SENSOR_POLL_CODE */ 

	/* Poll sensors once and dump data on terminal */
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		HAL_UART_Transmit( &huart1,
                           &num_sensor_bytes,
                           sizeof( num_sensor_bytes ), 
                           HAL_DEFAULT_TIMEOUT );

		/* Get the sensor readings */
	    sensor_subcmd_status = sensor_dump( &sensor_data_byte[0] );	

		/* Transmit sensor readings to PC */
		if ( sensor_subcmd_status == SENSOR_OK )
			{
			// readings_to_bytes( &sensor_readings_bytes[0], 
            //                    &sensor_readings[0] );
			HAL_UART_Transmit( &huart1                   , 
                               &sensor_data_byte[0]      , 
                               sizeof( sensor_data_byte ), 
                               HAL_SENSOR_TIMEOUT );
			return ( sensor_subcmd_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/* Subcommand not recognized */
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS      accel_status;
IMU_STATUS      gyro_status;
IMU_STATUS      mag_status;
IMU_DATA        *pIMU_data,IMU_data;               /*Initialize IMU structure*/
pIMU_data     = &IMU_data;                         /*Initialize pointer to IMU structure*/
uint16_t        dummy_baro_pressure;
uint16_t        dummy_baro_temp;
uint32_t        start;
start         = HAL_GetTick();
/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/
// TODO: Pass the sensor_data_ptr -> imu_data
accel_status         = imu_get_accel_xyz( &sensor_data_ptr->imu_data ); 
gyro_status          = imu_get_gyro_xyz( &sensor_data_ptr->imu_data );
mag_status           = imu_get_mag_xyz( &sensor_data_ptr->imu_data );

// TODO: Implement the actual get baro values
sensor_data_ptr->dummy_baro_pressure = dummy_baro_pressure;
sensor_data_ptr->dummy_baro_temp     = dummy_baro_temp;

sensor_data_ptr->time                = HAL_GetTick() - start;

// TODO: Implement memcpy sensor_data struct to pSensor_buffer (if read through USB)
// memcpy(pSensor_buffer, sensor_data, SENSOR_DATA_SIZE);

/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
if ( accel_status != IMU_TIMEOUT &&
     gyro_status  != IMU_TIMEOUT &&
     mag_status   != IMU_TIMEOUT  )
    {
        return SENSOR_OK;
    }
    else
    {
        return SENSOR_IMU_FAIL;
    }
} /* sensor_dump */


