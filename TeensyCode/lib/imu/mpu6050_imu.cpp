#include "mpu6050_imu.h"

/*
*   PURPOSE: 
*   This is *another* mpu6050 library, mainly because all of the more simple libraries are gpl 
*   liscensed, and I wanted something that was more C oriented rather that cpp. 
*
*   It's important to visit the invernsense datasheet register map here: 
*   https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf(if link doesnt work search for mpu6050 register datasheet)
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

/*!
*   @brief Configures the behavior of the interrupt pins and sets up the i2c bypassing
*   @note For more information, visit page 26 of the primary pdf. 
*/
#define INT_PIN_CFG      0x37

/*!
*   @brief Sets up the interrupt generation via interrupt sources. 
*   @note For more information, visit page 27 of the primay pdf. 
*/
#define INT_ENABLE       0x38

/*!
*   @brief Shows the interrupt status of each interrupt generation source. 
*   @note After reading the bitmask, the associated bits are cleared. 
*   @note For more information, take a look at page 28 of the primary pdf. 
*/
#define INT_STATUS       0x3A

/*!
*   @brief These registers all allow us to fetch the most recent values of information from the mpu6050. 
*   @note For implementation purposes, just call the accel_xout_h register, and since that's the first register
*   @note if you read out all 6 bytes, you'll get the all the accelerometer data. 
*   @note For more information, take a look at page 29 of the primary pdf. 
*/
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40

/*!
*   @brief These registers all allow us to fetch the most recent values of information from the mpu6050. 
*   @note For implementation purposes, just call the accel_xout_h register, and since that's the first register
*   @note if you read out all 6 bytes, you'll get the all the gyroscopeo data. 
*   @note For more information, take a look at page 31 of the primary pdf. 
*/
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
*   @note Page 45 of the primary pdf may have more information, but it's currently unknown 
*/
#define WHO_AM_I 0x75 

uint8_t device_address; 

mpu_init_status_t init_mpu6050(uint8_t i2c_address = 0x68); 
imu_data_raw get_latest_mpu6050_data(bool blocking); 
static void i2c_read_bytes(uint8_t sub_addr, uint8_t count, uint8_t *dest); 
static uint8_t i2c_read_byte(uint8_t sub_addr); 
static void i2c_write_byte(uint8_t sub_addr, uint8_t data); 

/*!
*   @brief Easy setup of the imu module. 
*   @note Default i2c address is 0x68, specify otherwise
*   @param uint8_t i2c_address
*   @return mpu_init_status_t status of setting up the imu
*/
mpu_init_status_t init_mpu_6050(uint8_t i2c_address = 0x68){
    device_address = i2c_address; 
    
    if(i2c_read_byte(WHO_AM_I) != 0x68)
        return MPU6050_NOT_FOUND; 
    
    i2c_write_byte(PWR_MGMT_1, 0x01); 


    return MPU6050_INIT_SUCCESS; 
}

/*!
*   @brief Getting the latest data from the imu
*   @param bool Whether or not we will wait for data to be available or not
*/
imu_data_raw get_latest_mpu6050_data(bool blocking){
    imu_data_raw dat; 
    return dat; 
}

/*!
*   @brief Helper function that allows us to fetch a certain amount of bytes on the i2c bus. 
*   @param uint8_t sub_addr, or register that we are writing from 
*   @param uint8_t count, how many bytes are we reading
*   @param uint8_t *dest, pointer to destination arr. 
*/
static void i2c_read_bytes(uint8_t sub_addr, uint8_t count, uint8_t *dest){
    Wire.beginTransmission(device_address); 
    Wire.write(sub_addr); 
    Wire.endTransmission(false); 

    uint8_t n = 0; 
    Wire.requestFrom(device_address, count); 
    while(Wire.available()){
        dest[n++] = Wire.read(); 
    }
}

/*!
*   @brief Helper function that lets us get a single byte.
*   @param uint8_t sub_addr, or register we are reading from 
*   @return uint8_t return_byte
*/
static uint8_t i2c_read_byte(uint8_t sub_addr){
    uint8_t dat; 
    i2c_read_bytes(sub_addr, 1, &dat); 
    return dat; 
}

/*!
*   @brief Helper function that let's us write bytes to specific registers 
*   @param uint8_t sub_addr, or register we are writing to 
*   @param uint8_t data that we want to right
*/
static void i2c_write_byte(uint8_t sub_addr, uint8_t data){
    Wire.beginTransmission(device_address); 
    Wire.write(sub_addr); 
    Wire.write(data); 
    Wire.endTransmission(); 
}