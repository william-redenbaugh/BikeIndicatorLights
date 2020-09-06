#include <Arduino.h> 
#include "OS/OSThreadKernel.h"
#include "MatrixEngine.hpp"

void setup() {
  os_init();   
  setup_matrix_engine(250);  
}

void loop() {
  matrix_fill_stop();
  os_thread_delay_s(1); 
  matrix_fill_black();
  os_thread_delay_s(1); 
}