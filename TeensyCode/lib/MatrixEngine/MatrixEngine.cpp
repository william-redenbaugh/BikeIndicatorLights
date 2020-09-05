#include "MatrixEngine.hpp"

// Pointer to matrix array data. 
static rgb24 *matrix_arr_ptr; 

// If we want to completely change the frame, we can do that here. 
static rgb24 matrix_buffer[matrix_width * matrix_height];

void setup_matrix_engine(uint8_t brightness){
    // Startup matrix driver code 
    setup_matrix(); 
    // Set to latest matrix brightness
    adjust_matrix_brightness(brightness); 

    // Getting the latest matrix pointer from the own
    matrix_arr_ptr = get_matrix_ptr(); 
    // Zero out everything in the buffer. 
    memset(matrix_buffer, 0, sizeof(matrix_buffer));
}

void set_matrix(uint8_t x, uint8_t y, rgb24 col){
    // Which spot in the array are we modifying. 
    matrix_arr_ptr[y * matrix_width + x] = col;
}

void set_matrix_buff(uint8_t x, uint8_t y, rgb24 col){
    // Which spot in the array are we modifying. 
    matrix_buffer[y * matrix_width + x] = col;
}

void set_matrix_hsv(uint8_t x, uint8_t y, HsvColor col){
    rgb24 rgb_col = HsvToRgb(col); 
    // Ignore compiler warning, these are in two different base formats
    // But look the same since their data types are the same
    set_matrix(x, y, rgb_col);
}