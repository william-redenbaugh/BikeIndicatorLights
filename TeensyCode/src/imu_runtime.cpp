#include "imu_runtime.hpp"

/*!
*   @brief Statically allocated thread stack. 
*/
static uint32_t imu_thread_stack[2048]; 

void setup_imu_runtime(void);  
static void imu_thread(void *parameters); 

/*!
*   @brief IMU runtime module initialization and construction
*/
void setup_imu_runtime(void){
    // Setting up a thread to deal all of our imu thread stuff on 
    os_add_thread(&imu_thread, NULL, sizeof(imu_thread_stack), imu_thread_stack); 
}

/*!
*   @brief IMU runtime thread
*/
static void imu_thread(void *parameters){

    init_mpu6050(MPU6050_DEFAULT_I2C_ADDR, ACCELEROMETER_4G, GYRO_500_DEGREE_SECCOND); 
    for(;;){
        os_thread_delay_ms(200);
        //imu_data_raw raw_dat = get_latest_mpu6050_data(false); 
    }
}