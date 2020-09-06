#ifndef _MATRIX_ENGINE_HPP
#define _MATRIX_ENGINE_HPP

#include <Arduino.h>

#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"
#include "OS/OSThreadKernel.h"
#include "HUB75.h"
#include "hsv_rgb_conv.hpp"

void setup_matrix_engine(uint8_t brightness);
void set_matrix(uint8_t x, uint8_t y, rgb24 col);
void set_matrix_buff(uint8_t x, uint8_t y, rgb24 col);
void set_matrix_hsv(uint8_t x, uint8_t y, HsvColor col);
void matrix_swipe_right(uint16_t speed); 
void matrix_swipe_left(uint16_t speed); 
void matrix_fill_stop(void); 
void matrix_fill_black(void); 

#endif 