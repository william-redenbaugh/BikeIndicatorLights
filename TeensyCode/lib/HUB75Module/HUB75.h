#ifndef _HUB75_H
#define _HUB75_H

// Including primary arduino libraries 
#include <Arduino.h> 

// Including our RTOS scheduling libraries 
#include "OS/OSThreadKernel.h"
#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"

// Our FlexIO PWM Libraries
#include "FlexIO_t4.h"
// Gamma Lookup table librarie 
#include "gammaLUT.h"
// DMA peripheral library
#include "DMAChannel.h"

typedef struct rgb24 {
    rgb24() : rgb24(0,0,0) {}
    rgb24(uint8_t r, uint8_t g, uint8_t b) {
        red = r; green = g; blue = b;
    }
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb24;

typedef struct rgb48 {
    rgb48() : rgb48(0,0,0) {}
    rgb48(uint16_t r, uint16_t g, uint16_t b) {
        red = r; green = g; blue = b;
    }
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} rgb48;


extern void setup_matrix(void); 

#endif 