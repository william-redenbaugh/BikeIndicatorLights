#include "imu_runtime.hpp"

#include "led_strip_runtime.h"

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

    init_mpu6050(MPU6050_DEFAULT_I2C_ADDR, ACCELEROMETER_16G, GYRO_500_DEGREE_SECCOND); 

    for(;;){
        os_thread_sleep_ms(10);

        int n = millis();        
        // We get 20 samples of data. 
        imu_data_raw lat_data = get_latest_mpu6050_data_sampled(20);    

        Serial.println((millis() - n));  
        accel_data_g dat = translate_accel_raw_g(lat_data); 

        bike_led_signal_state_t state = current_matrix_bike_animation();

        if(dat.a_z < -.1){
            if(dat.a_z >= -.3){
                if(state != BIKE_LED_SIGNAL_STOP){
                    trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_STOP);
                }
            }
            else{
                if(state != BIKE_LED_SIGNAL_STOP_FAST){
                    trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_STOP_FAST); 
                }
            }
        }
        else
            if(state != BIKE_LED_SIGNAL_WHITE)
                trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_WHITE);
    }
}
