#include "led_strip_runtime.h"

/*!
*   @brief Statically allocated thread stack. 
*/
static uint32_t matrix_thread_stack[8192]; 

/*!
*   @brief Drawing and manipulation memory. 
*/
static byte matrix_drawing_memory[NUM_STRIP_LEDS * 3]; 

/*!
*   @brief DMA memory buffer that we will use to write to the LEDs
*/
static DMAMEM byte matrix_display_memory[NUM_STRIP_LEDS * 12]; 

/*!
*   @brief WS2812b strip manipulation object. 
*/  
static WS2812Serial matrix = WS2812Serial(NUM_STRIP_LEDS, matrix_display_memory, matrix_drawing_memory, STRIP_LED_GPIO, WS2812_GRB);

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
static const rgbw_t RGBW_WHITE =    {100, 100, 100, 0}; 
static const rgbw_t RGBW_RED =      {255, 0, 0, 20}; 
static const rgbw_t RGBW_BLACK =    {0, 0, 0, 0}; 

/*!
*   @brief General purpose function declarations 
*/
extern void trigger_matrix_bike_animation(bike_led_signal_state_t signal);
void start_led_strip_runtime(void);
static void matrix_thread(void *parameters); 
static void start_led_strip(void); 
static void update_matrix(void); 
static inline void set_matrix(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b); 
static inline void draw_white(void); 
static inline void fill_col_nodraw(rgbw_t col); 
static inline void fill_col_draw(rgbw_t col);

/*!
*   @brief Animation function declarations
*/
static inline void stop(void); 
static void stop_fast(void); 
static void turn_left(void); 
static void turn_right(void); 
static void turn_left_stop(void); 
static void turn_right_stop(void); 

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
    os_add_thread(&matrix_thread, NULL, sizeof(matrix_thread_stack), matrix_thread_stack); 
}

/*!
*   @brief Strip thread function
*/
static void matrix_thread(void *parameters){
    
    // If there is an issue with setting up the led strip, we just sleep the thread. 
    if(!matrix.begin())
        while(1)
            os_thread_delay_s(1);        

    matrix.setBrightness(8); 

    // Sensor starts off doing nothing in theory
    turn_right();

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
        turn_left(); 
        break; 

        case(BIKE_LED_SIGNAL_TURN_LEFT_STOP):
        turn_left_stop(); 
        break; 
        
        case(BIKE_LED_SIGNAL_TURN_RIGHT):
        turn_right(); 
        break; 
        
        case(BIKE_LED_SIGNAL_TURN_RIGHT_STOP):
        turn_right_stop(); 
        break; 
        
        default: 
        break;
        }

        os_thread_delay_ms(10); 
    }
}

/*!
*   @brief Starts up the matrix manipulation module
*/
static void start_led_strip(void){
    // If there is an issue with setting up the led strip, we just sleep the thread. 
    if(!matrix.begin())
        while(1)
            os_thread_delay_s(1);        
}

/*!
*   @brief Updates the matrix.
*/
static void update_matrix(void){
    matrix.show(); 
}

/*!
*   @brief Allows us to set the values of the matrix on a cartesiean axis. 
*   @param uint8_t x position
*   @param uint8_t y position
*   @param uint8_t r color value
*   @param uint8_t g color value 
*   @param uint8_t b color value 
*/
static inline void set_matrix(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b){
    if(x >= 8 || y >= 16)
        return; 

    // Set the position
    int pos = 127-(8 * y + x);
    // Write to pixel buffer. 
    matrix.setPixel(pos, r, g, b); 
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
        matrix.setPixelColor(n, col.r, col.g, col.b); 
}

static inline void fill_col_draw(rgbw_t col){
    fill_col_nodraw(col); 
    matrix.show(); 
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

        if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 100))
            return;  
        
        fill_col_draw(RGBW_BLACK);

        if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 100))
            return;  
    }
}

/*!
*   @brief Animation that let's us turn left
*/
static void turn_left(void){
    for(;;){
        for(int x = 7; x >= 0; x--){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 100, 100, 100); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
        for(int x = 7; x >= 0; x--){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 255,165,0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
    }
}

/*!
*   @brief Animation that let's us turn right
*/
static void turn_right(void){
    for(;;){
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 100, 100, 100); 

            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;     
            update_matrix(); 
        }
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 255,165,0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
    }
}

/*!
*   @brief Animation that let's us turn left while also signaling stop
*/
static void turn_left_stop(void){
    for(int x = 7; x >= 0; x--){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 250, 0, 0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
        for(int x = 7; x >= 0; x--){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 255,165,0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
}

/*!
*   @brief Animation that let's us turn right while also signaling stop
*/
static void turn_right_stop(void){
    for(;;){
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 250, 0, 0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
        for(int x = 0; x < 8; x++){
            for(int y = 0; y < 16; y++)
                set_matrix(x, y, 255,165,0); 
            if(bike_trigger_signal.wait_n_clear(THREAD_SIGNAL_0, 50))
                return;  
            update_matrix(); 
        }
    }
}
