// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"

// Matrix Helper Libraries 
#include "MatrixEngine.hpp"

// IMU runtime module 
#include "imu_runtime.hpp"

void setup() {
  os_init();   
  setup_matrix_engine(250);  
  Serial.begin(115200);
  void setup_imu_runtime(); 
}

void loop() {
  os_thread_delay_ms(100);
}