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
uint32_t      sensor_readings[ NUM_SENSORS ]; /* Readings obtained from each 
                                                 sensor                       */
uint8_t       sensor_readings_bytes[ 4*NUM_SENSORS ];
const uint8_t num_sensor_bytes = 4*NUM_SENSORS; /* Number of bytes to be 
                                                   transmitted back to PC     */

// IMU data size should be in header
// imu data, imu data size, imu data byte list, memcpy the addr of firts imu data byte and imu data, imu data size

uint8_t      sensor_data_byte[ IMU_DATA_SIZE ]    /*Initialize sensor data byte array
                                                     with size of all sensor data*/
const uint8_t num_sensor_bytes = IMU_DATA_SIZE;
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
for ( uint8_t i = 0; i < num_sensor_bytes; ++i )
	{
    sensor_data_byte[i] = 0;
    }

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
*       reads from all sensors and transmits data back to host PC              *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    uint32_t* pSensor_buffer /* Pointer to buffer where sensor data should 
                                be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS accel_status;
IMU_STATUS gyro_status;
IMU_STATUS mag_status;
IMU_DATA      *pIMU_data,IMU_data;                 /*Initialize IMU structure*/
pIMU_data     = &IMU_data;                         /*Initialize pointer to IMU structure*/
/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

accel_status         = imu_get_accel_xyz( pIMU_data );
gyro_status          = imu_get_gyro_xyz( pIMU_data );
mag_status           = imu_get_mag_xyz( pIMU_data );

memcpy( pSensor_buffer, pIMU_DATA , IMU_DATA_SIZE)

/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
if ( accel_status != IMU_TIMEOUT &&
     gyro_status  != IMU_TIMEOUT &&
     mag_status   != IMU_TIMEOUT  )
    {
        return SENSOR_OK
    }
    else
    {
        return SENSOR_IMU_FAIL
    }
} /* sensor_dump */


