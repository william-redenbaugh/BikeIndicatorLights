#include "imu_runtime.hpp"

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
    
    for(;;){

    }
}