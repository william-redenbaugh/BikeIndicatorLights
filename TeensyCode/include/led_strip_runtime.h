#ifndef _LED_STRIP_RUNTIME_H
#define _LED_STRIP_RUNTIME_H

#include <Arduino.h> 

#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"
#include "OS/OSThreadKernel.h"

#include "hsv_rgb_conv.hpp"
#include "WS2812Serial.h"

/*!
*   @brief Enumerated animation values that will let us trigger the LED strip to do different stuff. 
*/
typedef enum{
    BIKE_LED_SIGNAL_STOP, 
    BIKE_LED_SIGNAL_STOP_FAST,
    BIKE_LED_SIGNAL_WHITE, 
    BIKE_LED_SIGNAL_TURN_LEFT, 
    BIKE_LED_SIGNAL_TURN_LEFT_STOP,
    BIKE_LED_SIGNAL_TURN_RIGHT,
    BIKE_LED_SIGNAL_TURN_RIGHT_STOP
}bike_led_signal_state_t; 

/*!
*   @brief How many LEDs does the project have. 
*/
const int NUM_STRIP_LEDS = 128;  

/*!
*   @brief What gpio pad is our LED strip on. 
*/
const int STRIP_LED_GPIO = 20; 

void start_led_strip_runtime(void); 
extern void trigger_led_strip_bike_animation(bike_led_signal_state_t signal);

#endif