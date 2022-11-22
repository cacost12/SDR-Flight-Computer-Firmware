/*******************************************************************************
*
* FILE: 
* 		flash.c
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
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
#include "flash.h"
#include "led.h"
#include "sdr_pin_defines_A0002.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

//wip
extern SPI_HandleTypeDef hspi2;               /* SPI handle                   */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

static void address_to_bytes
    (
        uint32_t address,
        uint8_t* address_bytes
    )
{
    address_bytes[0] = address & 0xFF;
    address_bytes[1] = (address >> 8) & 0xFF;
    address_bytes[2] = (address >> 16) & 0xFF;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a flash subcommand based on input from the sdec terminal      *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_cmd_execute
	(
    uint8_t             subcommand   ,
	HFLASH_BUFFER*      pflash_handle,
	UART_HandleTypeDef* huart
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint8_t          opcode;                    /* Subcommand opcode              */
uint8_t          num_bytes;                 /* Number of bytes on which to 
                                               operate                        */
uint8_t          status;                    /* Return value of UART API calls */
uint8_t          erase_size;                /* Opcode of (4k/32k/64k) block 
                                               erase                          */

/*------------------------------------------------------------------------------
 Command Input processing 
------------------------------------------------------------------------------*/
opcode    = ( subcommand & FLASH_SUBCMD_OP_BITMASK ) >>  5;
num_bytes = ( subcommand & FLASH_NBYTES_BITMASK    ); 

/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/

switch(opcode)
	{
    /* READ Subcommand */
    case FLASH_SUBCMD_READ:
        {
        /* Get Addres bits */
		status = HAL_UART_Receive(
                                 huart                             , 
                                 &( pflash_handle -> address[0] )  , 
                                 sizeof( pflash_handle -> address ), 
                                 HAL_DEFAULT_TIMEOUT
                                 );	
        /* WIP, assign address[3] to address_32 whenever it's updated.*/
        if (status != HAL_TIMEOUT )
			{
			    /* Call API Function */
				status = flash_read( pflash_handle, num_bytes );
				if (status == FLASH_TIMEOUT)
					{
                    /* Bytes not read */
				    return FLASH_TIMEOUT;
                    }

                /* Bytes read successfully into pbuffer */
                for (int i = 0; i < num_bytes; ++i)
				    {
                    uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
				    status = HAL_UART_Transmit(
                                            huart               , 
                                            pbuffer             , 
                                            sizeof( uint8_t )   , 
                                            HAL_DEFAULT_TIMEOUT
                                            );
                    if (status == HAL_TIMEOUT)
                        {
                        /* Bytes not transimitted */
                        return FLASH_TIMEOUT;	
                        }
				    else
                        {
                        /* Do nothing, UART working fine */
                        }
                    }
			}
		else    	
			{
			/* Address not recieved */
			return FLASH_TIMEOUT;
            }
        
        /* Bytes read and transimitted back sucessfully*/
	    return FLASH_OK;	
        }

    /* HS READ Subcommand */
    case FLASH_SUBCMD_HS_READ:
        {
        /* Get Addres bits */
		status = HAL_UART_Receive(
                                 huart                             , 
                                 &( pflash_handle -> address[0] )  , 
                                 sizeof( pflash_handle -> address ), 
                                 HAL_DEFAULT_TIMEOUT
                                 );	
        if (status != HAL_TIMEOUT )
			{
			    /* Call API Function */
				status = flash_high_speed_read( pflash_handle, num_bytes );
				if (status == FLASH_TIMEOUT)
					{
                    /* Bytes not read */
				    return FLASH_TIMEOUT;
                    }

                /* Bytes read successfully into pbuffer */
                for (int i = 0; i < num_bytes; ++i)
				    {
                    uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
				    status = HAL_UART_Transmit(
                                            huart               , 
                                            pbuffer             , 
                                            sizeof( uint8_t )   , 
                                            HAL_DEFAULT_TIMEOUT
                                            );
                    if (status == HAL_TIMEOUT)
                        {
                        /* Bytes not transimitted */
                        return FLASH_TIMEOUT;	
                        }
				    else
                        {
                        /* Do nothing, UART working fine */
                        }
                    }
			}
		else    	
			{
			/* Address not recieved */
			return FLASH_TIMEOUT;
            }
        
        /* Bytes read and transimitted back sucessfully */
	    return FLASH_OK;	
        }

    /* ENABLE Subcommand */
    case FLASH_SUBCMD_ENABLE:
        {
		status = flash_write_enable( pflash_handle );
		return status;
        }

    /* DISABLE Subcommand */
    case FLASH_SUBCMD_DISABLE:
        {
		status = flash_write_disable( pflash_handle );
		return status;
        }

    /* WRITE Subcommand */
    case FLASH_SUBCMD_WRITE:
        {
		/* Get Address bits */
		status = HAL_UART_Receive(
                                 huart                             , 
                                 &( pflash_handle -> address[0] )  , 
                                 sizeof( pflash_handle -> address ), 
                                 HAL_DEFAULT_TIMEOUT
                                 );
		if (status != HAL_TIMEOUT )
			{
			/* Get bytes to be written to flash */
			for (int i = 0; i < num_bytes; ++i)
				{
				uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
				status = HAL_UART_Receive(
                                         huart              , 
                                         pbuffer            , 
                                         sizeof( uint8_t )  , 
                                         HAL_DEFAULT_TIMEOUT
                                         );
				if (status == HAL_TIMEOUT)
					{
					/* Bytes not received */
				    return FLASH_TIMEOUT;	
                    }
				else
					{
					/* Do nothing, UART working fine */
                    }
				}
            }
		else    	
			{
			/* Address not recieved */
			return FLASH_TIMEOUT;
            }

		/* Call API function */
		status = flash_write( pflash_handle );
        if( status == FLASH_TIMEOUT )
            {
            return FLASH_TIMEOUT;
            }
        else if( status == FLASH_WRITE_PROTECTED)
            {
            return FLASH_WRITE_PROTECTED;
            }

	    return FLASH_OK;	
        }

    /* ERASE Subcommand */
    case FLASH_SUBCMD_ERASE:
        {
        /* No Prerequiste data from/to UART*/
        /* Call API Function*/
	    status = flash_erase( pflash_handle );
        if( status == FLASH_TIMEOUT )
            {
            return FLASH_TIMEOUT;
            }
        else if( status == FLASH_WRITE_PROTECTED)
            {
            return FLASH_WRITE_PROTECTED;
            }

        return FLASH_OK;	
        }

    /* BLOCK ERASE Subcommand*/
    case FLASH_SUBCMD_64K_ERASE:
        erase_size = FLASH_OP_HW_64K_ERASE;
    case FLASH_SUBCMD_32K_ERASE:
        if(erase_size != 0)
            erase_size = FLASH_OP_HW_32K_ERASE;
    case FLASH_SUBCMD_4K_ERASE:
        {
            if(erase_size != 0)
                erase_size = FLASH_OP_HW_4K_ERASE;
        
        /* Get address bits */
		status = HAL_UART_Receive(
                                 huart                             , 
                                 &( pflash_handle -> address[0] )  , 
                                 sizeof( pflash_handle -> address ), 
                                 HAL_DEFAULT_TIMEOUT
                                 );
        if (status != HAL_TIMEOUT )
			{
			    /* Call API Function*/
	            status = flash_block_erase( pflash_handle, num_bytes, erase_size );
                if( status == FLASH_TIMEOUT )
                {
                    return FLASH_TIMEOUT;
                }
                else if( status == FLASH_WRITE_PROTECTED)
                {
                    return FLASH_WRITE_PROTECTED;
                }

			}
		else    	
			{
			/* Address not recieved */
			return FLASH_TIMEOUT;
            }

        return FLASH_OK;	
        }

    /* STATUS Subcommand */
    case FLASH_SUBCMD_STATUS:
        {
		/* Call API function */
		status = flash_status( pflash_handle );
		if ( status == FLASH_TIMEOUT )
			{
            return FLASH_TIMEOUT;
            }

		/* Send status register contents back to PC */
		HAL_UART_Transmit( 
                         huart                                     , 
                         &( pflash_handle -> status_register )     ,
                         sizeof( pflash_handle -> status_register ),
                         HAL_DEFAULT_TIMEOUT 
                         );
	    return FLASH_OK;	
        }

    /* Unrecognized subcommand code: invoke error handler */
	default:
        {
	    return FLASH_UNRECOGNIZED_OP;	
        }
    }
} /* flash_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_status                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the status register of the flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_status
	(
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status;     /* Status code returned by hal spi functions          */
uint8_t transmit_data;  /* Data to be transmitted over SPI                    */

/*------------------------------------------------------------------------------
 Initialiazations 
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
transmit_data = FLASH_OP_HW_RDSR;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select line low */
HAL_GPIO_WritePin(
				 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_RESET
                 );

/* Send RDSR code to flash chip */
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Recieve status code */
hal_status = HAL_SPI_Receive(
                            &hspi2                                    ,
                            &( pflash_handle      -> status_register ),
                            sizeof( pflash_handle -> status_register ),
							HAL_DEFAULT_TIMEOUT
                            );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Drive slave select line high */
HAL_GPIO_WritePin(
                 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_SET
                 );

/* Flash status register read successful */
return FLASH_OK;

} /* flash_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Enable writing to the external flash chip                              *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_write_enable
    (
    HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
transmit_data = FLASH_OP_HW_WREN;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select pin low */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_RESET    
                  );

/* Transmit WREN code to flash over SPI */
hal_status = HAL_SPI_Transmit(
                             &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT
                             );


/* Drive slave select pin high */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_SET    
                  );

/* Set WP MCU pin to HIGH */
//HAL_GPIO_WritePin(FLASH_WP_GPIO_PORT, FLASH_WP_PIN, GPIO_PIN_SET);


/* Set write enabled bit in flash buffer handle */
if ( hal_status != HAL_TIMEOUT )
	{
	pflash_handle -> write_enabled = FLASH_WP_WRITE_ENABLED;
	return FLASH_OK;
    }
else
	{
	return FLASH_TIMEOUT;
    }

} /* flash_write_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Disable writing to the external flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_write_disable
    (
    HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
transmit_data = FLASH_OP_HW_WRDI;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select pin low */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_RESET    
                  );

/* Transmit WREN code to flash over SPI */
hal_status = HAL_SPI_Transmit(
                             &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT
                             );

/* Drive slave select pin high */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_SET    
                  );

/* Set WP MCU pin to LOW */
//HAL_GPIO_WritePin(FLASH_WP_GPIO_PORT, FLASH_WP_PIN, GPIO_PIN_RESET);

/* Reset the write enabled bit in flash buffer handle */
if ( hal_status != HAL_TIMEOUT )
	{
	pflash_handle -> write_enabled = FLASH_WP_READ_ONLY;
	return FLASH_OK;
    }
else
	{
	return FLASH_TIMEOUT;
    }

} /* flash_write_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       writes bytes from a flash buffer to the external flash                 *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_write 
    (
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
/* Return data from SPI bus */
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data = FLASH_OP_HW_BYTE_PROGRAM; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if(pflash_handle -> write_enabled == false)
    return FLASH_WRITE_PROTECTED;

/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Send command code  */
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Send address bytes */
hal_status = HAL_SPI_Transmit(
                             &hspi2                            ,
                             &( pflash_handle -> address[0] )  ,
                             sizeof( pflash_handle -> address ),
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Write bytes */
hal_status = HAL_SPI_Transmit(
                             &hspi2                         ,
                             pflash_handle -> pbuffer       ,
                             pflash_handle -> num_bytes     ,
                             FLASH_WRITE_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;

} /* flash_write */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_read                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads a specified number of bytes using a flash buffer                 *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    )
{

uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data = FLASH_OP_HW_READ; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/
/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Send command code*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                             ,
                             &( pflash_handle -> address[0] )   ,
                             sizeof( pflash_handle -> address )  ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Recieve output into buffer*/
for(int i = 0; i < num_bytes; ++i){
	uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
    hal_status = HAL_SPI_Receive(
                                &hspi2                    ,
                                pbuffer                   ,
                                sizeof( uint8_t )         ,
							    HAL_DEFAULT_TIMEOUT
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
	return FLASH_TIMEOUT;
    }
}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;

} /* flash_read */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_erase                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       erases the entire flash chip                                           *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    )
{

uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data = FLASH_OP_HW_FULL_ERASE; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if(pflash_handle -> write_enabled == false)
{
    return FLASH_WRITE_PROTECTED;
}

/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Full chip erase */
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;

} /* flash_erase */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_high_speed_read                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       high speed reads a specified number of bytes using a flash buffer      *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_high_speed_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    )
{
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data = FLASH_OP_HW_READ_HS; /* Data to be transmitted over SPI                     */
uint8_t dummy_byte = 0;
uint8_t *dummy_ptr = &dummy_byte;

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/
/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Send command code*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                             ,
                             &( pflash_handle -> address[0] )   ,
                             sizeof( pflash_handle -> address )  ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Send Dummy Byte */
hal_status = HAL_SPI_Transmit(
							 &hspi2              ,
                             dummy_ptr           ,
                             sizeof( uint8_t )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Recieve output into buffer*/
for(int i = 0; i < num_bytes; ++i){
	uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
    hal_status = HAL_SPI_Receive(
                                &hspi2                    ,
                                pbuffer                   ,
                                sizeof( uint8_t )         ,
							    HAL_DEFAULT_TIMEOUT
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
	return FLASH_TIMEOUT;
    }
}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;

} /* flash_high_speed_read */


FLASH_CMD_STATUS flash_extract(HFLASH_BUFFER* pflash_handle){
    uint8_t hal_status;    /* Status code return by hal spi functions             */
    uint8_t transmit_data = FLASH_OP_HW_READ_HS; /* Data to be transmitted over SPI                     */
    uint8_t dummy_byte = 0;
    uint8_t *dummy_ptr = &dummy_byte;

    /*------------------------------------------------------------------------------
    API function implementation
    ------------------------------------------------------------------------------*/
    /* Drive chip enable line low */
    HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

    /* Send command code*/
    hal_status = HAL_SPI_Transmit(
                                &hspi2                    ,
                                &transmit_data            ,
                                sizeof( transmit_data )   ,
                                HAL_DEFAULT_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    address_to_bytes(pflash_handle->address_32, pflash_handle->address);
    /* Send Address*/
    hal_status = HAL_SPI_Transmit(
                                &hspi2                             ,
                                &( pflash_handle -> address[0] )   ,
                                sizeof( pflash_handle -> address )  ,
                                HAL_DEFAULT_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    /* Send Dummy Byte */
    hal_status = HAL_SPI_Transmit(
                                &hspi2              ,
                                dummy_ptr           ,
                                sizeof( uint8_t )   ,
                                HAL_DEFAULT_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    /* Recieve output into buffer*/
    hal_status = HAL_SPI_Receive(
                                &hspi2                    ,
                                pflash_handle->pbuffer    ,
                                sizeof( uint8_t )         ,
                                HAL_DEFAULT_TIMEOUT
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
    return FLASH_TIMEOUT;
    }
    

    /* Drive chip enable line high */
    HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

    return FLASH_OK;

}

FLASH_CMD_STATUS flash_store
    (
        HFLASH_BUFFER* pflash_handle,
        SENSOR_DATA* sensor_data_ptr,
        uint32_t time               
    )
{
        /*------------------------------------------------------------------------------
    Local variables 
    ------------------------------------------------------------------------------*/
    /* Return data from SPI bus */
    uint8_t hal_status;    /* Status code return by hal spi functions             */
    uint8_t transmit_data = FLASH_OP_HW_BYTE_PROGRAM; /* Data to be transmitted over SPI                     */
    uint8_t buffer[30];
    /*------------------------------------------------------------------------------
    API function implementation
    ------------------------------------------------------------------------------*/
    
    //convert address_32 to bytes
    address_to_bytes(pflash_handle->address_32, pflash_handle->address);

    //get Data in sensor_handle to uint8_t array to pbuffer
    memcpy(&time, &buffer[0], sizeof(uint32_t));
    memcpy(sensor_data_ptr, &buffer[4], sizeof(SENSOR_DATA));

    pflash_handle->pbuffer = &buffer[0];

    /* Check if write_enabled */
    if(pflash_handle -> write_enabled == false)
        return FLASH_WRITE_PROTECTED;

    /* Drive chip enable line low */
    HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

    /* Send command code  */
    hal_status = HAL_SPI_Transmit(
                                &hspi2                    ,
                                &transmit_data            ,
                                sizeof( transmit_data )   ,
                                HAL_DEFAULT_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    /* Send address bytes */
    hal_status = HAL_SPI_Transmit(
                                &hspi2                            ,
                                &( pflash_handle -> address[0] )  ,
                                sizeof( pflash_handle -> address ),
                                HAL_DEFAULT_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    /* Write bytes */
    hal_status = HAL_SPI_Transmit(
                                &hspi2                         ,
                                pflash_handle -> pbuffer       ,
                                pflash_handle -> num_bytes     ,
                                FLASH_WRITE_TIMEOUT 
                                );

    if ( hal_status == HAL_TIMEOUT )
    {
        return FLASH_TIMEOUT;
    }

    /* Drive chip enable line high */
    HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

    return FLASH_OK;

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_block_erase                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       block erase 4k/32k/64k of data from Flash chip                         *
*                                                                              *
*******************************************************************************/
FLASH_CMD_STATUS flash_block_erase
    (
    HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes    ,
    uint8_t        erase_size
    )
{
    
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data = erase_size; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if(pflash_handle -> write_enabled == false)
    return FLASH_WRITE_PROTECTED;

/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Send command code */
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                             ,
                             &( pflash_handle -> address[0] )   ,
                             sizeof( pflash_handle -> address )  ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
{
	return FLASH_TIMEOUT;
}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;
} /* flash_block_erase */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
