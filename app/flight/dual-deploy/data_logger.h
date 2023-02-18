/*******************************************************************************
*
* FILE: 
* 		data_logger.h
*
* DESCRIPTION: 
* 	    Contains procedures for logging data onto the flight computer's 
*       external flash chip	
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DATA_LOGGER_H 
#define DATA_LOGGER_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/
#include "main.h"


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Address of flash headers */
#define FLASH_HEADER_ADDRESS    ( 0x000000 )
#define FLASH_HEADER1_ADDRESS   ( 0x000000 )
#define FLASH_HEADER2_ADDRESS   ( 0x001000 )

/* Flash header info */
#define FLASH_HEADER_SIZE       ( 4*1024   ) /* 4kB */

/* Flash valid states */
#define FLASH_HEADER_VALID      ( 0x00 )
#define FLASH_HEADER_INVALID    ( 0x10 )


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Altimeter program settings struct */
typedef struct _ALT_PROG_SETTINGS
    {
    uint32_t main_alt;     /* Main Parachute deployment altitude    */
    uint32_t drogue_delay; /* Delay of drogue ejection after apogee */
    } ALT_PROG_SETTINGS;

/* Flash header */
typedef struct _FLASH_HEADER
    {
    uint8_t           valid;                /* Set to indicate valid header  */
    ALT_PROG_SETTINGS alt_prog_settings;    /* Altimeter dual-deploy config  */
    uint32_t          flight_events[16][3]; /* History of flight events      */
    uint8_t           num_flights;          /* Number of flights in memory   */
    uint8_t           next_flight_pos;      /* Location of oldest flight in 
                                               memory                        */
    uint32_t          checksum;             /* Checksum for error correction */
    } FLASH_HEADER;

/* Status return codes */
typedef enum _DATA_LOG_STATUS
    {
    DATA_LOG_OK                ,
    DATA_LOG_INVALID_CHECKSUM1 , /* First header checksum invalid      */ 
    DATA_LOG_INVALID_CHECKSUM2 , /* Second header checksum invalid     */
    DATA_LOG_INVALID_CHECKSUMS , /* Both headers checksum invalid      */
    DATA_LOG_HEADERS_NOT_EQUAL , /* Headers not equal                  */
    DATA_LOG_FLASH_ERROR       , /* Flash API doesn't return correctly */
    DATA_LOG_HEADER1_INVALID   , /* Primary header invalid             */
    DATA_LOG_HEADER2_INVALID   , /* Backup header invalid              */
    DATA_LOG_HEADERS_INVALID     /* Both headers invalid               */
    } DATA_LOG_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Load the flash headers from external flash */
DATA_LOG_STATUS data_logger_load_header
    (
    void
    );


/* Load the flash headers with the default configuration */
DATA_LOG_STATUS data_logger_init_header
    (
    void
    );

/* Compute the header checksums and verify validity */
DATA_LOG_STATUS data_logger_check_header
    (
    void
    );

/* Sets the contents of the flash header */
DATA_LOG_STATUS data_logger_update_header
    (
    void
    );

/* Sets the main parachute deployment altitude and drogue delay by writing to 
   the flight computer's external flash */
void program_altimeter 
    (
    ALT_PROG_SETTINGS alt_prog_settings
    );

/* Updates the flash header with data from the most recent flight */
void record_flight_events
    (
    void
    );

#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGER_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/