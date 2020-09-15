// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"


// IMU runtime module 
#include "imu_runtime.hpp"
#include "led_strip_runtime.h"
#include "MODULES/HM18/hm18.h"

#include "MODULES/PROTOCALLBACKS/teensy_coms.h"

void setup() {
  os_init();   
  Serial.begin(115200);

  // Setting up the IMU
  setup_imu_runtime(); 

  // Setting up the led strip
  start_led_strip_runtime();
   
  start_hm18(&Serial1); 

  // Setup message callbacks with the desired device. 
  message_callbacks_begin(&Serial1, 9600);
}

void loop() {
  os_thread_delay_s(1);
}