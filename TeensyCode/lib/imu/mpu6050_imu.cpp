#include "mpu6050_imu.h"

/*
*   PURPOSE: 
*   This is *another* mpu6050 library, mainly because all of the more simple libraries are gpl 
*   liscensed, and I wanted something that was more C oriented rather that cpp. 
*
*   It's important to visit the invernsense datasheet register map here: 
*   https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
*   It's got everything you need to understand all of the registers outline below(but I will provide a brief description)
*/

/*!
*   @brief Sample Rate Divider register. 
*   @note Sets the divider that determines the sample rate of the device. 
*   @note Equation is (default_sample_rate)/(sample_rate + 1).
*   @note Default sample rate of gyroscope is 8khz, and a 1khz accelerometer rate
*   @note Just pass in the divider as the next byte to write to the i2c slave after this register. 
*/
#define SMPLRT_DIV       0x19

/*!
*   @brief Configures the digital low pass filter(DLPF) and the Frame synchronization pin sampling. 
*   @brief For both the accelerometer and the gyroscope.
*   @note To configure the these features, it's essentially a bitmask. 
*   @note To see the official bitmask, visit page 13 of the primary PDF. 
*/
#define CONFIG           0x1A

/*!
*   @brief Configures the Gyroscope reading rate and self tests. 
*   @note Check the enumerated gyroscope values to see the available ranges in the header file
*   @note For more information, check page 14 of the primary PDF. 
*/
#define GYRO_CONFIG      0x1B

/*!
*   @brief Configures the accelorometer reading rates and self tests. 
*   @note Check the enumerated accelerometer values to see the available ranges in the header file
*   @note For more information, check page 15 of the primary pdf. 
*/
#define ACCEL_CONFIG     0x1C
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define PWR_MGMT_1       0x6B 
#define PWR_MGMT_2       0x6C

/*!
*   @brief Register that helps identify what type of device we are talking to on the i2c bus 
*   @note Should return 0x68 if it's the correct device. 
*/
#define WHO_AM_I 0x75 

// Just in case we want to use this somewhere else. 
#undef WHO_AM_I