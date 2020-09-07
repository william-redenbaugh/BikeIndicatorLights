#ifndef MPU6050_IMU_H
#define MPU6050_IMU_H

#include <Arduino.h> 

#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"

/*!
*   @brief Enumerated accelerometer ranges defined so  we can send the bounds of the device. 
*/
typedef enum{
    GYRO_250_DEGREE_SECCOND = 0, 
    GYRO_500_DEGREE_SECCOND, 
    GYRO_1000_DEGREE_SECCOND, 
    GYRO_2000_DEGREE_SECCOND
}mpu_accelerometer_range_t; 

/*!
*   @brief Enumerated accelerometer ranges defined so  we can send the bounds of the device. 
*/  
typedef enum{
    ACCELEROMETER_2G,
    ACCELEROMETER_4G,
    ACCELEROMETER_8G,
    ACCELEROMETER_16G
}mpu_gyro_range_t; 

/*!
*   @brief Struct that holds the raw imu data. 
*/
typedef struct{
    // Accelerometer Data. 
    uint16_t a_x; 
    uint16_t a_y; 
    uint16_t a_z; 

    // Gyroscope data. 
    uint16_t g_x; 
    uint16_t g_y; 
    uint16_t g_z; 
}imu_data_raw;

void init_mpu6050(uint8_t i2c_address); 
imu_data_raw get_latest_mpu6050_data(bool blocking); 

#endif 