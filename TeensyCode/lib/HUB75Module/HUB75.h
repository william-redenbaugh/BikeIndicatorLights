#ifndef _HUB75_H
#define _HUB75_H

// Including primary arduino libraries 
#include <Arduino.h> 

// Our FlexIO PWM Libraries
#include "FlexIO_t4.h"
// Gamma Lookup table librarie 
#include "gammaLUT.h"
// DMA peripheral library
#include "DMAChannel.h"

#include "hsv_rgb_conv.hpp"

typedef struct rgb48 {
    rgb48() : rgb48(0,0,0) {}
    rgb48(uint16_t r, uint16_t g, uint16_t b) {
        red = r; green = g; blue = b;
    }
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} rgb48;

static const int matrix_width = 64;               // Width of the overall display
static const int matrix_height = 32;              // Height of the overall display
static const int matrix_panel_height = 32;        // Height of the individual panels making up the display

// Call this to start matrix drivers. 
extern void setup_matrix(void); 
// Pointer to matrix array. 
extern rgb24* get_matrix_ptr(void); 
// Allows us to change matrix. 
extern void adjust_matrix_brightness(uint8_t brightness); 
#endif 