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

#include "DS_HELPER/priority_queue.hpp"

PriorityQueuePointerHeap node; 

void setup() {
  os_init();   
  
  Serial.begin(115200);

  // Setting up the IMU
  // setup_imu_runtime(); 

  // Setting up the led strip
  start_led_strip_runtime();

  // Starting up our low priority work thread. 
  //setup_lwip_thread(); 

  // Start up the bluetooth module.    
  // start_hm18(&Serial1); 

  // Setup message callbacks with the desired device. 
  // message_callbacks_begin(&Serial1, 9600);

  // We want to periodically check the battery. 
  // init_battery_checking_runtime();

  os_thread_delay_s(3);
  node.init_priority_queue(45); 
}

void loop() {
  Serial.println("Putting items on priority queue: "); 
  for(int n = 0; n < 6; n++){
    int k = random(255); 
    int l = random(255);

    Serial.print("Priority: "); 
    Serial.println(k); 
    Serial.print("Key: "); 
    Serial.println(l); 
    
    node.insert((void*)l, k); 
  }

  Serial.println("Taking items off priority queue");
  while(1){
    PriorityQueueHeapNode *curr_node = node.peek_top_node(); 
    if(curr_node == NULL)
      break; 
      
    Serial.print("Priority: "); 
    Serial.println(curr_node->priority); 

    Serial.print("Value: "); 
    Serial.println((int)curr_node->ptr); 
    node.pop(); 
  }

  os_thread_delay_s(4);
}