/*******************************************************************************
*
* FILE: 
* 		baro.h
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BARO_H 
#define BARO_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return codes for API functions */
typedef enum BARO_STATUS
	{
	BARO_OK                     ,
	BARO_FAIL                   , 
	BARO_TIMEOUT                ,
	BARO_UNRECOGNIZED_HAL_STATUS,
	BARO_I2C_ERROR
	} BARO_STATUS;

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* I2C Device Params */
#define BARO_I2C_ADDR	    ( 0x76 << 1 )	/* 1110110 -> 0x76 */


/* Barometric Pressure Sensor register addresses */
#define BARO_REG_CHIP_ID		( 0x00      )
#define BARO_PRESSURE_DATA  	( 0x04		)  
#define BARO_TEMPERATURE_DATA	( 0x05		) 


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* read one register for BARO */
BARO_STATUS BARO_Read_Register
    (
    uint8_t reg_addr, 
    uint8_t *pData
    );

/* read multiple registers for BARO */
BARO_STATUS BARO_Read_Registers
    (
    uint8_t reg_addr, 
    uint8_t *pData, 
    uint8_t num_registers
    );

/* write a register to BARO */
BARO_STATUS BARO_Write_Register
    (
    uint8_t reg_addr, 
    uint8_t *pData
    );

/* verifies sensor can be accessed */
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id 
	);


/* gets pressure data from sensor */
BARO_STATUS baro_get_pressure
	(
    uint32_t baro_pressure_data
	);

/* gets temp data from sensor */
BARO_STATUS baro_get_temp
	(
    uint32_t baro_temp_data
	);

/* converts pressure and temp data into altitude --> do research on formula */
BARO_STATUS baro_get_altitude
	(
    void
	);


#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
