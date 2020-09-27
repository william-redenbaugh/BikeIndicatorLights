// Our primary libraries
#include <Arduino.h> 
#include "OS/OSThreadKernel.h"

// IMU runtime module 
#include "imu_runtime.hpp"

// LED strip runtime module 
#include "led_strip_runtime.h"

// Bluetooth module 
#include "MODULES/HM18/hm18.h"

// Google Protobuffer callback modules. 
#include "MODULES/PROTOCALLBACKS/teensy_coms.h"

// Low priorty thread callback module. 
#include "MODULES/LPWORK/lp_work_thread.h"

// Voltage read module
#include "HAL/VOLTAGE_READ/voltage_read.h"

// Battery checking runtime module
#include "battery_checking_runtime.hpp"

void setup() {
  os_init();   
  
  Serial.begin(115200);

  // Setting up the IMU
  setup_imu_runtime(); 

  // Setting up the led strip
  start_led_strip_runtime();

  // Starting up our low priority work thread. 
  setup_lwip_thread(); 

  // Start up the bluetooth module.    
  start_hm18(&Serial1); 

  // Setup message callbacks with the desired device. 
  message_callbacks_begin(&Serial1, 9600);

  // We want to periodically check the battery. 
  init_battery_checking_runtime();
}

void loop() {

  os_thread_delay_s(1);
}