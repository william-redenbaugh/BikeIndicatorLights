// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"

// Matrix Helper Libraries 
#include "MatrixEngine.hpp"

// IMU runtime module 
#include "imu_runtime.hpp"
#include "mpu6050_imu.h"
void setup() {
  os_init();   

  // Setting up the IMU
  setup_imu_runtime(); 

  setup_matrix_engine(250);  
  Serial.begin(115200);
  
  setup_imu_runtime();
}

void loop() {
  os_thread_delay_ms(100);
}