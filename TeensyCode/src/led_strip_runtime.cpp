#include "led_strip_runtime.h"

/*!
*   @brief Statically allocated thread stack. 
*/
static uint32_t led_strip_thread_stack[8192]; 

/*!
*   @brief Drawing and manipulation memory. 
*/
static byte led_strip_drawing_memory[NUM_STRIP_LEDS * 4]; 

/*!
*   @brief DMA memory buffer that we will use to write to the LEDs
*/
static DMAMEM byte led_strip_display_memory[NUM_STRIP_LEDS * 16]; 

/*!
*   @brief WS2812b strip manipulation object. 
*/  
static WS2812Serial led_strip = WS2812Serial(NUM_STRIP_LEDS, led_strip_display_memory, led_strip_drawing_memory, STRIP_LED_GPIO, WS2812_GRBW);

/*!
*   @brief Next bike signal state. We employ the last-man-wins stratagy

*/
static volatile bike_led_signal_state_t next_bike_led_state; 

/*!
*   @brief Bitfield that let's thread know there is a new 
*/
static OSSignal bike_trigger_signal; 

/*!
*   @brief Function declarations 
*/

extern void trigger_led_strip_bike_animation(bike_led_signal_state_t signal);
void start_led_strip_runtime(void);
static void led_strip_thread(void *parameters); 
static inline void draw_white(void); 
static inline void signal_orange(uint16_t k); 
static inline void stop(void);
static void swipe_left(void); 
static void swipe_right(void); 

/*!
*   @brief Triggers the bike signal animation to change
*/
extern void trigger_led_strip_bike_animation(bike_led_signal_state_t signal){
    next_bike_led_state = signal; 
    bike_trigger_signal.signal(THREAD_SIGNAL_0);    
}

/*!
*   @brief Strip generation constructor. 
*/
void start_led_strip_runtime(void){
    os_add_thread(&led_strip_thread, NULL, sizeof(led_strip_thread_stack), led_strip_thread_stack); 
}

/*!
*   @brief Strip thread function
*/
static void led_strip_thread(void *parameters){
    
    // If there is an issue with setting up the led strip, we just sleep the thread. 
    if(!led_strip.begin())
        while(1)
            os_thread_delay_s(1);        

    led_strip.setBrightness(20); 

    // Sensor starts off doing nothing in theory
    draw_white(); 

    // Runtime loop
    for(;;){
        switch(next_bike_led_state){
        case(BIKE_LED_SIGNAL_STOP):
        stop(); 
        break;

        case(BIKE_LED_SIGNAL_WHITE):
        draw_white(); 
        break; 

        case(BIKE_LED_SIGNAL_TURN_LEFT):
        swipe_left(); 
        break; 

        case(BIKE_LED_SIGNAL_TURN_RIGHT):
        swipe_right(); 
        break; 
        default: 
        break;
        }
    }
}

/*!
*   @brief Sets whole strip white. 
*/
static inline void draw_white(void){
    for(int n = 0; n < NUM_STRIP_LEDS; n++)
        led_strip.setPixelColor(n, 0, 0, 0, 70);

    led_strip.show(); 
    // We sit and wait until we recieve the command to start looking for the next animation
    bike_trigger_signal.wait_notimeout(THREAD_SIGNAL_0);
    bike_trigger_signal.clear(THREAD_SIGNAL_0); 
}

/*!
*   @brief Helper function that helps with signaling the stuff.  
*/
static inline void signal_orange(uint16_t k){
    
    for(int n = 0; n < NUM_STRIP_LEDS; n+= 20)
        led_strip.setPixelColor((n + k) % NUM_STRIP_LEDS, 255, 151, 0, 0);
    
    for(int n = 20; n < NUM_STRIP_LEDS; n+= 20)
        led_strip.setPixelColor((n + k) % NUM_STRIP_LEDS, 0, 0, 0, 70);
    
    led_strip.show(); 
}

/*!
*   @brief Sets whole strip to red
*/
static inline void stop(void){
    for(int n = 0; n < NUM_STRIP_LEDS; n++)
        led_strip.setPixelColor(n, 255, 0, 0, 20);

    led_strip.show();
    // We sit and wait until we recieve the command to start looking for the next animation
    bike_trigger_signal.wait_notimeout(THREAD_SIGNAL_0);
    bike_trigger_signal.clear(THREAD_SIGNAL_0); 
}

/*!
*   @brief Helper function that allows us to see the strip to swipe left
*/
static void swipe_left(void){
    for(;;){
        for(int n = NUM_STRIP_LEDS + 30; n >= 30; n--){
            signal_orange(n); 
            // If there is a trigger to change the animation
            if(bike_trigger_signal.wait(THREAD_SIGNAL_0, 10)){
                // Then we clear the flag and yeet out of this animation
                bike_trigger_signal.clear(THREAD_SIGNAL_0); 
                return;       
            }
        }
    }
}

/*!
*   @brief Helper function that allows us to see the strip to swipe right
*/
static void swipe_right(void){
    for(int n = 0; n < NUM_STRIP_LEDS; n++){
        signal_orange(n); 
        // If there is a trigger to change the animation
        if(bike_trigger_signal.wait(THREAD_SIGNAL_0, 10)){
            // Then we clear the flag and yeet out of this animation
            bike_trigger_signal.clear(THREAD_SIGNAL_0); 
            return;       
        }    
    }
}