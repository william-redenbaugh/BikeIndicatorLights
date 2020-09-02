#include <Arduino.h> 
#include "OS/OSThreadKernel.h"

void setup(void){
  os_init();
}

void loop(void){
  os_thread_delay_s(1); 
}