// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"


// IMU runtime module 
#include "imu_runtime.hpp"
#include "matrix_runtime.h"
void setup() {
  os_init();   

  // Setting up the IMU
  setup_imu_runtime(); 

  // Setting up matrix
  start_matrix_runtime(); 

  Serial.begin(115200);
}

void loop() {
  os_thread_delay_ms(100);
}