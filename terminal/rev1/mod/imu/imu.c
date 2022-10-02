/*******************************************************************************
*
* FILE: 
* 		imu.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/
/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"

/*------------------------------------------------------------------------------
 Initialize struct for IMU 
------------------------------------------------------------------------------*/
IMU_DATA imu *pIMU_data,IMU_data; 
pIMU_data       = &IMU_data;

/*------------------------------------------------------------------------------
 Default config for IMU 
------------------------------------------------------------------------------*/
IMU_CONFIG imu_config *pimu_config1,imu_config1;  /* Initialize IMU config structure */
pimu_config1 = &imu_config1;                      /* Set a pointer to IMU config structure */      

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_MAG_Read_Register                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read one register from magnetometer module in the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_Status IMU_MAG_Read_Register
    (
    uint8_t reg,
    uint8_t *data
    )
{
return HAL_I2C_Mem_Read
        (
        hi2c1,
        IMU_MAG_ADDR, 
        reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        1, 
        HAL_MAX_DELAY
        );
} /* IMU_MAG_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_MAG_Read_Registers                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from magnetometer`
        module in the IMU                                                         *
*                                                                              *
*******************************************************************************/
IMU_Status IMU_MAG_Read_Registers
    (
    uint8_t reg,
    uint8_t *data, 
    uint8_t length
    )
{
return HAL_I2C_Mem_Read
        (
        hi2c1, 
        IMU_MAG_ADDR, 
        reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        length, 
        HAL_MAX_DELAY
        );
} /* IMU_MAG_Read_Registers */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Read_Register                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read one register from acceleration and gyroscope module in the IMU                                                           *
*                                                                              *
*******************************************************************************/
IMU_Status IMU_Read_Register
    (
    uint8_t reg, 
    uint8_t *data
    )
{
return HAL_I2C_Mem_Read
        (
        hi2c1, 
        IMU_ADDR, 
        reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        1, 
        HAL_MAX_DELAY
        );
} /* IMU_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Read_Registers                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from acceleration
        and gyroscope module in the IMU                                   *
*                                                                              *
*******************************************************************************/
IMU_Status IMU_Read_Registers
    (
    uint8_t reg, 
    uint8_t *data, 
    uint8_t length
    )
{
return HAL_I2C_Mem_Read
        (
        hi2c1, 
        IMU_ADDR, 
        reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        length, 
        HAL_MAX_DELAY
        );
} /* IMU_Read_Registers */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Write_Register                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Write one register to the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_Status IMU_Write_Register
    (
    uint8_t reg, 
    uint8_t *data
    )
{
return HAL_I2C_Mem_Write
        (
        hi2c1, 
        IMU_ADDR, 
        reg, 
        I2C_MEMADD_SIZE_8BIT, 
        data, 
        1, 
        HAL_MAX_DELAY
        );
} /* IMU_Write_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_accel_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z acceleration values from the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_DATA imu *imu_get_accel_xyz
    (
        IMU_DATA imu *thisIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regAccelX,regAccelY,regAccelZ;
uint8_t     errorCounter=0;
IMU_Status  status;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read ACCEL_X high byte and low byte registers
status                  = IMU_Read_Registers(ACCEL_XOUT_H, regAccelX,2);
errorCounter            = errorCounter + (status!=HAL_OK);
// Read ACCEL_Y high byte and low byte registers
status                  = IMU_Read_Registers(ACCEL_YOUT_H, regAccelY,2);
errorCounter            = errorCounter + (status!=HAL_OK);
// Read ACCEL_Z high byte and low byte registers
status                  = IMU_Read_Registers(ACCEL_ZOUT_H, regAccelZ,2);
errorCounter            = errorCounter + (status!=HAL_OK);

// Combine high byte and low byte to 16 bit data 
uint16_t accel_x_raw    = ((uint16_t)regAccelX[0]<<8) | regAccelX[1];
uint16_t accel_y_raw    = ((uint16_t)regAccelY[0]<<8) | regAccelY[1];
uint16_t accel_z_raw    = ((uint16_t)regAccelZ[0]<<8) | regAccelZ[1];

// Convert 16 bit to m/s^2
thisIMU->accel_x        = (accel_x_raw/pimu_config1->accel_setting*9.8)*65536;
thisIMU->accel_y        = (accel_y_raw/pimu_config1->accel_setting*9.8)*65536;
thisIMU->accel_z        = (accel_z_raw/pimu_config1->accel_setting*9.8)*65536;

// if it returns 0, it means it is working properly.
if (errorCounter==0){
    return thisIMU;
}
} /* imu_get_accel_xyz */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_gryo_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z gyro values from the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_DATA imu *imu_get_gryo_xyz
    (
    IMU_DATA imu *thisIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regGyroX,regGyroY,regGyroZ;
uint8_t     errorCounter=0;
IMU_Status  status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read GYRO_X high byte and low byte registers
status              = IMU_Read_Registers(GYRO_XOUT_H, regGyroX,2);
errorCounter        = errorCounter + (status!=HAL_OK);
// Read GYRO_Y high byte and low byte registers
status              = IMU_Read_Registers(GYRO_YOUT_H, regGyroY,2);
errorCounter        = errorCounter + (status!=HAL_OK);
// Read GYRO_Z high byte and low byte registers
status              = IMU_Read_Registers(GYRO_ZOUT_H, regGyroZ,2);
errorCounter        = errorCounter + (status!=HAL_OK);

// Combine high byte and low byte to 16 bit data 
uint16_t gyro_x_raw = ((uint16_t)regGyroX[0]<<8) | regGyroX[1];
uint16_t gyro_y_raw = ((uint16_t)regGyroY[0]<<8) | regGyroY[1];
uint16_t gyro_z_raw = ((uint16_t)regGyroZ[0]<<8) | regGyroZ[1];

// Convert 16 bit to usable gyro data
thisIMU->gyro_x        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;
thisIMU->gyro_y        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;
thisIMU->gyro_z        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;

// if it returns 0, it means it is working properly.
if (errorCounter==0){
    return thisIMU;
}
} /* imu_get_gyro_xyz */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_mag_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z magnetometer values from the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_DATA imu *imu_get_mag_xyz
    (
    IMU_DATA imu *thisIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regMagX,regMagY,regMagZ;
uint8_t     errorCounter=0;
IMU_Status status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read MAG_X high byte and low byte registers
status              = IMU_MAG_Read_Registers(MAG_XOUT_H, regMagX,2);
errorCounter        = errorCounter + (status!=HAL_OK);
// Read MAG_Y high byte and low byte registers
status              = IMU_MAG_Read_Registers(MAG_YOUT_H, regMagY,2);
errorCounter        = errorCounter + (status!=HAL_OK);
// Read MAG_Z high byte and low byte registers
status              = IMU_MAG_Read_Registers(MAG_ZOUT_H, regMagZ,2);
errorCounter        = errorCounter + (status!=HAL_OK);

// Combine high byte and low byte to 16 bit data 
uint16_t mag_x_raw  = ((uint16_t)regMagX[0]<<8) | regMagX[1];
uint16_t mag_y_raw  = ((uint16_t)regMagY[0]<<8) | regMagY[1];
uint16_t mag_z_raw  = ((uint16_t)regMagZ[0]<<8) | regMagZ[1];

// Convert 16 bit to usable gyro data
thisIMU->mag_x      = (mag_x_raw/pimu_config1->mag_setting)*65536;
thisIMU->mag_y      = (mag_x_raw/pimu_config1->mag_setting)*65536;
thisIMU->mag_z      = (mag_x_raw/pimu_config1->mag_setting)*65536;

// if it returns 0, it means it is working properly.
if (errorCounter==0){
    return thisIMU;
}
} /* imu_get_mag_xyz */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_temp                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        temperature from the IMU                                               *
*                                                                              *
*******************************************************************************/
IMU_DATA imu *imu_get_temp
    (
    IMU_DATA imu *thisIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t regData;
uint8_t errorCounter=0;
uint8_t RoomTemp_Offset;
uint8_t Temp_Sensitivity;
uint8_t temp_degC,temp_degF;
IMU_Status status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read temperature high byte and low byte registers
status              = IMU_Read_Registers(TEMP_OUT_H, regData,2);

// Combine high byte and low byte to 16 bit data 
errorCounter        = errorCounter + (status!=HAL_OK);
uint16_t raw_temp   = ((uint16_t) regData[0]<<8) | regData[1];
temp_degC           = ((raw_temp - RoomTemp_Offset)/Temp_Sensitivity)+21;
temp_degF           = (temp_degC*9/5)+32;
// if it returns 0, it means it is working properly.
if (errorCounter==0){
    thisIMU->temp   =temp_degF;
    return thisIMU;
}
} /* imu_get_temp */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_device_id                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		return the device ID of the IMU to verify that the 
*       IMU registers are accessible                                           *
*                                                                              *
*******************************************************************************/
 
uint8_t imu_get_device_id
    (
    IMU_DATA imu *thisIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t regData;
uint8_t errorCounter=0;
IMU_Status status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read Device ID register
status          = IMU_Read_Register(IMU_ID, regData);
errorCounter    = errorCounter + (status!=HAL_OK);
if (regData!=IMU_ID){
    return 255;
}
// if it returns 0, it means it is working properly.
if (errorCounter==0){
    // return device ID
    return regData;
} else {
    return errorCounter;
}
} /* imu_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_config                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Change configuration of accel, gyro, mag                                          *
*                                                                              *
*******************************************************************************/

void IMU_Config_Func
    (
    IMU_CONFIG imu_config *this_imu_config,
    uint8_t  this_accel_setting;
    uint16_t this_gyro_setting;
    uint16_t this_mag_setting;
    )
{
this_imu_config->accel_setting = this_accel_setting;
this_imu_config->gyro_setting  = this_gyro_setting;
this_imu_config->mag_setting   = this_mag_setting;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_cmd_exe                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Execute sensor subcommand                                           *
*                                                                              *
*******************************************************************************/

IMU_DATA imu *sensor_cmd_exe
    (
        uint8_t subcommand
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t IMU_recieved_data;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
switch (subcommand){
    case (IMU_DUMP_CODE):
    {
        pIMU_data           = imu_get_accel_xyz(pIMU_data);
        pIMU_data           = imu_get_gyro_xyz(pIMU_data);
        pIMU_data           = imu_get_mag_xyz(pIMU_data);
        IMU_received_data   = pIMU_data;
        break;
    }
    case (IMU_POLL_CODE):
    {
        // TODO: Implement poll code
        break;
    }
}
// Return data recieved executed by subcommands
return IMU_recieved_data;
}