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
*   @brief RGBW struct that helps make code more readable
*/
typedef struct{
    uint8_t r; 
    uint8_t g; 
    uint8_t b; 
    uint8_t w; 
}rgbw_t; 

/*!
*   @brief Some color definitions to keep handy. 
*/
static const rgbw_t RGBW_ORANGE =   {255, 151, 0, 0}; 
static const rgbw_t RGBW_WHITE =    {0, 0, 0, 70}; 
static const rgbw_t RGBW_RED =      {255, 0, 0, 20}; 
static const rgbw_t RGBW_BLACK =    {0, 0, 0, 0}; 

/*!
*   @brief Function declarations 
*/
extern void trigger_led_strip_bike_animation(bike_led_signal_state_t signal);
void start_led_strip_runtime(void);
static void led_strip_thread(void *parameters); 
static inline void draw_white(void); 
static inline void fill_col_nodraw(rgbw_t col); 
static inline void fill_col_draw(rgbw_t col);
static inline void stop(void);
static void stop_fast(void); 
static void swipe_left(rgbw_t foreground, rgbw_t background); 
static void swipe_right(rgbw_t foreground, rgbw_t background); 

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

    led_strip.setBrightness(10); 

    // Sensor starts off doing nothing in theory
    
    swipe_left(RGBW_ORANGE, RGBW_RED); 

    // Runtime loop
    for(;;){
        switch(next_bike_led_state){
        case(BIKE_LED_SIGNAL_STOP):
        stop(); 
        break;

        case(BIKE_LED_SIGNAL_STOP_FAST):
        stop_fast(); 
        break; 

        case(BIKE_LED_SIGNAL_WHITE):
        draw_white(); 
        break; 

        case(BIKE_LED_SIGNAL_TURN_LEFT):
        swipe_left(RGBW_ORANGE, RGBW_WHITE); 
        break; 

        case(BIKE_LED_SIGNAL_TURN_LEFT_STOP):
        swipe_left(RGBW_ORANGE, RGBW_RED); 
        break; 
        
        case(BIKE_LED_SIGNAL_TURN_RIGHT):
        swipe_right(RGBW_ORANGE, RGBW_WHITE); 
        break; 
        
        case(BIKE_LED_SIGNAL_TURN_RIGHT_STOP):
        swipe_right(RGBW_ORANGE, RGBW_RED); 
        break; 
        
        default: 
        break;
        }

        os_thread_delay_ms(10); 
    }
}

/*!
*   @brief Sets whole strip white. 
*/
static inline void draw_white(void){
    fill_col_draw(RGBW_WHITE); 
 
    // We sit and wait until we recieve the command to start looking for the next animation
    bike_trigger_signal.wait_notimeout(THREAD_SIGNAL_0);
    bike_trigger_signal.clear(THREAD_SIGNAL_0); 
}

/*!
*   @brief Helper function that sets the entire strip to a specific color
*/
static inline void fill_col_nodraw(rgbw_t col){
    for(int n = 0; n < NUM_STRIP_LEDS; n++)
        led_strip.setPixelColor(n, col.r, col.g, col.b, col.w); 
}

static inline void fill_col_draw(rgbw_t col){
    fill_col_nodraw(col); 
    led_strip.show(); 
}

/*!
*   @brief Helper function that helps with signaling the stuff.  
*/
static inline void signal_helper_right(uint16_t k, rgbw_t foreground, rgbw_t background){
    
    for(int n = 0; n < NUM_STRIP_LEDS; n+= 14)
        led_strip.setPixelColor((n + k) % NUM_STRIP_LEDS, foreground.r, foreground.g, foreground.b, foreground.w);
    
    for(int n = 14; n < NUM_STRIP_LEDS; n+= 14)
        led_strip.setPixelColor((n + k) % NUM_STRIP_LEDS, background.r, background.g, background.b, background.w); 
    
    led_strip.show(); 
}


/*!
*   @brief Sets whole strip to red
*/
static inline void stop(void){
    fill_col_draw(RGBW_RED);

    // We sit and wait until we recieve the command to start looking for the next animation
    bike_trigger_signal.wait_notimeout(THREAD_SIGNAL_0);
    bike_trigger_signal.clear(THREAD_SIGNAL_0); 
}

static void stop_fast(void){
    for(;;){
        fill_col_draw(RGBW_RED); 

        if(bike_trigger_signal.wait(THREAD_SIGNAL_0, 100)){
            // Then we clear the flag and yeet out of this animation
            bike_trigger_signal.clear(THREAD_SIGNAL_0); 
            return;       
        }
        
        fill_col_draw(RGBW_BLACK);

        if(bike_trigger_signal.wait(THREAD_SIGNAL_0, 100)){
            // Then we clear the flag and yeet out of this animation
            bike_trigger_signal.clear(THREAD_SIGNAL_0); 
            return;       
        } 
    }
}

/*!
*   @brief Helper function that allows us to see the strip to swipe left
*/
static void swipe_left(rgbw_t forground, rgbw_t background){
    fill_col_nodraw(background); 
    for(;;){
        for(int n = NUM_STRIP_LEDS/2-1; n >= 0; n--){
            led_strip.setPixelColor(n, forground.r, forground.g, forground.b, forground.w);
            
            // If there is a trigger to change the animation
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 10))
                return;   

            led_strip.show(); 
        }
        
        if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 170))
            return; 

        // Fill back as default color
        for(int n = NUM_STRIP_LEDS/2-1; n >= 0; n--)
            led_strip.setPixelColor(n, background.r, background.g, background.b, background.w);
        led_strip.show();

        if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 170))
            return;   
    }
}

/*!
*   @brief Helper function that allows us to see the strip to swipe right
*/
static void swipe_right(rgbw_t forground, rgbw_t background){
    fill_col_nodraw(background); 
    for(;;){
        for(int n = NUM_STRIP_LEDS/2 - 1; n < NUM_STRIP_LEDS; n++){
            led_strip.setPixelColor(n, forground.r, forground.g, forground.b, forground.w);
            
            // If there is a trigger to change the animation
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 10))
                return;       
     
            led_strip.show(); 
        }

        if(bike_trigger_signal.wait(THREAD_SIGNAL_0, 170)){
            // Then we clear the flag and yeet out of this animation
            bike_trigger_signal.clear(THREAD_SIGNAL_0); 
            return;   
        }        

        // Fill back as default color
        for(int n = NUM_STRIP_LEDS/2 - 1; n < NUM_STRIP_LEDS; n++)
            led_strip.setPixelColor(n, background.r, background.g, background.b, background.w);
        led_strip.show();

        if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 170))
            return;   
    }
}