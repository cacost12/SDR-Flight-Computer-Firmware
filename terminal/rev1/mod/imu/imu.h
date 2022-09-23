/*******************************************************************************
*
* FILE: 
* 		imu.h
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

/*------------------------------------------------------------------------------
 Defines 
------------------------------------------------------------------------------*/
#define MPU9250_ADDR                0x68<<1
#define MPU9250_MAG_ADDR            0x0C<<1
#define MPU9250_ID                  0x71
#define MPU9250_POWER_MANAGEMENT    0x01
/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
#define GYRO_CONFIG                 0x1B
#define ACCEL_CONFIG                0x1C
#define ACCEL_XOUT_H                0x3B
#define ACCEL_XOUT_L                0x3C
#define ACCEL_YOUT_H                0X3D
#define ACCEL_YOUT_L                0x3E
#define ACCEL_ZOUT_H                0x3F
#define ACCEL_ZOUT_L                0x40
#define TEMP_OUT_H                  0x41
#define TEMP_OUT_L                  0x42
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0X45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48
#define MAG_XOUT_H                  0x04
#define MAG_XOUT_L                  0x03
#define MAG_YOUT_H                  0X06
#define MAG_YOUT_L                  0x05
#define MAG_ZOUT_H                  0x08
#define MAG_ZOUT_L                  0x07          
  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
// Structure for imu accel, gyro, mag and temp
struct imu{
    I2C_HandleTypeDef hi2c;
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double mag_x;
    double mag_y;
    double mag_z;
    double temp;
};

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef MPU9250_MAG_Read_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef MPU9250_MAG_Read_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef MPU9250_Read_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef MPU9250_Read_Registers(struct imu *thisIMU, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef MPU9250_Write_Register(struct imu *thisIMU, uint8_t reg, uint8_t *data);

struct imu_xyz *imu_get_accel_xyz(struct imu *thisIMU);

struct imu_xyz *imu_get_gryo_xyz(struct imu *thisIMU);

struct imu_xyz *imu_get_mag_xyz(struct imu *thisIMU);

struct imu_temp *imu_get_temp(struct imu *thisIMU);

uint8_t imu_get_device_id(struct imu *thisIMU);

#endif /* IMU_H */
