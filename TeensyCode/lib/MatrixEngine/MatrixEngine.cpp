#include "MatrixEngine.hpp"

/*!
*   @brief Pointer to the matrix array
*   @note Write to this array if you want instant change to the matrix
*   @note The size of the array is matrix_width * matrix_height
*/
static rgb24 *matrix_arr_ptr; 

/*!
*   @brief Call this to get the matrix stuff up and working!
*   @param uint8_t brightness(how bright do you want to make the display)
*/
void setup_matrix_engine(uint8_t brightness){
    // Startup matrix driver code 
    setup_matrix(); 
    // Set to latest matrix brightness
    adjust_matrix_brightness(brightness); 
    // Getting the latest matrix pointer from the own
    matrix_arr_ptr = get_matrix_ptr(); 
}

/*!
*   @brief Instantly set a specific LED on the matrix to a specificed rgb24
*   @param uint8_t x
*   @param uint8_t y
*   @param rgb24 struct color that we want the matrix
*/
void set_matrix(uint8_t x, uint8_t y, rgb24 col){
    // Which spot in the array are we modifying. 
    matrix_arr_ptr[y * matrix_width + x] = col;
}

/*!
*   @brief Instantly set a specific LED on the matrix to a specificed HsvColor 
*   @param uint8_t x
*   @param uint8_t y
*   @param HsvColor
*/
void set_matrix_hsv(uint8_t x, uint8_t y, HsvColor col){
    rgb24 rgb_col = HsvToRgb(col); 
    // Ignore compiler warning, these are in two different base formats
    // But look the same since their data types are the same
    set_matrix(x, y, rgb_col);
}

/*!
*   @brief Helper function that just deals with swiping right
*   @param rgb24 col(color we are swiping)
*   @param uint16_t speed
*/
inline static void swipe_right(rgb24 col, uint16_t speed){
    for(int x = 0; x < matrix_width; x++){
        for(int y = 0; y < matrix_height; y++)
            matrix_arr_ptr[y * matrix_width + x] = col; 
        os_thread_delay_ms(speed);
    }
}

/*!
*   @brief Helper function that just deals with swiping left
*   @param rgb24 col(color we are swiping)
*   @param uint16_t speed
*/
inline static void swipe_left(rgb24 col, uint16_t speed){
    for(int x = matrix_width; x >= 0; x--){
        for(int y = 0; y < matrix_height; y++)
            matrix_arr_ptr[y * matrix_width + x] = col;
        os_thread_delay_ms(speed);
    }
}

/*!
*   @brief Swipe the matrix right at a specific speed value 
*   @note Used to signal right on the matrix. 
*   @param speed (how much does system delay in ms)
*/
void matrix_swipe_right(uint16_t speed){
  rgb24 signal_orange = {253, 184, 19};
  swipe_right(signal_orange, speed);

  rgb24 signal_black = {0, 0, 0}; 
  swipe_right(signal_black, speed);
}

/*!
*   @brief Swipe the matrix right at a specific speed value 
*   @note Used to signal left on the matrix. 
*   @param speed (how much does system delay in ms)
*/
void matrix_swipe_left(uint16_t speed){
    rgb24 signal_orange = {253, 184, 19};
    swipe_left(signal_orange, speed);

    rgb24 signal_black = {0, 0, 0}; 
    swipe_left(signal_orange, speed);
}

/*!
*   @brief Helper function to set the whole matrix to a specific color
*   @param rgb24 col(which color we are changing the matrix to)
*/
static inline void matrix_fill(rgb24 col){
    for(int n = 0; n < matrix_width * matrix_height; n++)   
        matrix_arr_ptr[n] = col; 
}

/*!
*   @brief Set's matrix to red since that means stop
*/
void matrix_fill_stop(void){
  rgb24 red = {255, 0, 0};

  matrix_fill(red);
}

/*!
*   @brief Clear's out matrix to black 
*/
void matrix_fill_black(void){
  rgb24 black = {0, 0, 0};

  matrix_fill(black);
}