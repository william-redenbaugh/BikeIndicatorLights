// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"


// IMU runtime module 
#include "imu_runtime.hpp"
#include "matrix_runtime.h"
#include "MODULES/HM18/hm18.h"

void setup() {
  os_init();   
  Serial.begin(115200);

  // Setting up the IMU
  //setup_imu_runtime(); 

  // Setting up matrix
  //start_matrix_runtime(); 
  start_hm18(&Serial1); 
}

void loop() {
  os_thread_delay_ms(100);
}