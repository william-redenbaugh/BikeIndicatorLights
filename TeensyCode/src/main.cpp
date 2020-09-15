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
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_STOP); 
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_TURN_LEFT); 
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_TURN_RIGHT); 
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_TURN_LEFT_STOP); 
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_TURN_RIGHT_STOP); 
  os_thread_delay_s(3); 
  trigger_led_strip_bike_animation(BIKE_LED_SIGNAL_WHITE);
}