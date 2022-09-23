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

struct imu *myIMU=NULL;
myIMU = malloc(sizeof(struct imu));
myIMU->hi2c=hi2c1;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef MPU9250_MAG_Read_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(myIMU.hi2c, MPU9250_MAG_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_MAG_Read_Registers(struct imu *thisIMU, uint8_t reg, uint8_t *data, uint8_t length){
    return HAL_I2C_Mem_Read(myIMU.hi2c, MPU9250_MAG_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_Read_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(myIMU.hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_Read_Registers(struct imu *thisIMU, uint8_t reg, uint8_t *data, uint8_t length){
    return HAL_I2C_Mem_Read(myIMU.hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250_Write_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Write(myIMU.hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

struct imu *imu_get_accel_xyz(struct imu *thisIMU){
    uint8_t regData;
    uint8_t errorCounter=0;
    HAL_StatusTypeDef status;
    // Read ACCEL_X registers
    status = MPU9250_Read_Registers(thisIMU, ACCEL_XOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t accel_x_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read ACCEL_Y registers
    status = MPU9250_Read_Registers(thisIMU, ACCEL_YOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t accel_y_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read ACCEL_Z registers
    status = MPU9250_Read_Registers(thisIMU, ACCEL_ZOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t accel_z_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // if it returns 0, it means it is working properly.
    if (errorCounter==0){
        return thisIMU;
    }
}

struct imu *imu_get_gryo_xyz(struct imu *thisIMU){
    uint8_t regData;
    uint8_t errorCounter=0;
    HAL_StatusTypeDef status;
    // Read GYRO_X registers
    status = MPU9250_Read_Registers(thisIMU, GYRO_XOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t gyro_x_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read GYRO_Y registers
    status = MPU9250_Read_Registers(thisIMU, GYRO_YOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t gyro_y_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read GYRO_Z registers
    status = MPU9250_Read_Registers(thisIMU, GYRO_ZOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t gyro_z_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // if it returns 0, it means it is working properly.
    if (errorCounter==0){
        return thisIMU;
    }
}

struct imu *imu_get_mag_xyz(struct imu *thisIMU){
    uint8_t regData;
    uint8_t errorCounter=0;
    HAL_StatusTypeDef status;
    // Read MAG_X registers
    status = MPU9250_MAG_Read_Registers(thisIMU, MAG_XOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t mag_x_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read MAG_Y registers
    status = MPU9250_MAG_Read_Registers(thisIMU, MAG_YOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t mag_y_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // Read MAG_Z registers
    status = MPU9250_MAG_Read_Registers(thisIMU, MAG_ZOUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t mag_z_raw= ((uint16_t)regData[0]<<8) | regData[1];
    // if it returns 0, it means it is working properly.
    if (errorCounter==0){
        return thisIMU;
    }
}

struct imu *imu_get_temp(struct imu *thisIMU){
    uint8_t regData;
    uint8_t errorCounter=0;
    uint8_t RoomTemp_Offset;
    uint8_t Temp_Sensitivity;
    uint8_t temp_degC,temp_degF;
    HAL_StatusTypeDef status;
    // Read temperature registers
    status = MPU9250_Read_Registers(thisIMU, TEMP_OUT_H, regData,2);
    errorCounter = errorCounter + (status!=HAL_OK);
    uint16_t raw_temp = ((uint16_t) regData[0]<<8) | regData[1];
    temp_degC = ((raw_temp - RoomTemp_Offset)/Temp_Sensitivity)+21;
    temp_degF = (temp_degC*9/5)+32;
    // if it returns 0, it means it is working properly.
    if (errorCounter==0){
        thisIMU->temp=temp_degF;
        return thisIMU;
    }
}

// return the device ID of the IMU to verify that the IMU registers are accessible
uint8_t imu_get_device_id(struct imu *thisIMU){
    uint8_t regData;
    uint8_t errorCounter=0;
    HAL_StatusTypeDef status;
    status = MPU9250_Read_Register(MPU9250_ID, regData);
    errorCounter = errorCounter + (status!=HAL_OK);
    if (regData!=MPU9250_ID){
        return 255;
    }
    // if it returns 0, it means it is working properly.
    if (errorCounter==0){
        return regData;
    } else {
        return errorCounter;
    }
    
}