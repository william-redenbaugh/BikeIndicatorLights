/*
Author: William Redenbaugh
Last Edit Date: 7/3/2020
*/

#ifndef _HSV_RGB_CONV_HPP
#define _HSV_RGB_CONV_HPP

#include "Arduino.h"

typedef struct rgb24 {
    rgb24() : rgb24(0,0,0) {}
    rgb24(uint8_t r, uint8_t g, uint8_t b) {
        red = r; green = g; blue = b;
    }
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb24;

typedef struct HsvColor{
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HsvColor;

HsvColor RgbToHsv(rgb24 rgb);
rgb24 HsvToRgb(HsvColor hsv);

#endif 
