#include <Arduino.h> 
#include "OS/OSThreadKernel.h"
#include "HUB75.h"

void setup() {
  os_init();   
  setup_matrix(); 
  
}

void loop() {
  os_thread_delay_s(1);
}