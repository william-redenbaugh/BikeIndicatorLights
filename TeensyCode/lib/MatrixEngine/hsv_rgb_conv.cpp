/*
Author: William Redenbaugh
Last Edit Date: 7/20/2020
*/

#include "hsv_rgb_conv.hpp"

HsvColor RgbToHsv(rgb24 rgb);
rgb24 HsvToRgb(HsvColor hsv);

// CREDIT FOR HSV TO RGB CONVERSTION GOES TO THIS GUY: https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
// STACKOVERFLOW POSTERS ARE OUR OVERLOARDS. 

/**************************************************************************/
/*!
    @brief Converts RGB data into HSV data
    @param rgb struct. 
    @returns hsv struct. 
*/
/**************************************************************************/
HsvColor RgbToHsv(rgb24 rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.red < rgb.green ? (rgb.red < rgb.blue ? rgb.red : rgb.blue) : (rgb.green < rgb.blue ? rgb.green : rgb.blue);
    rgbMax = rgb.red > rgb.green ? (rgb.red > rgb.blue ? rgb.red : rgb.blue) : (rgb.green > rgb.blue ? rgb.green : rgb.blue);

    hsv.v = rgbMax;
    if (hsv.v == 0){
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0){
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.red)
        hsv.h = 0 + 43 * (rgb.green - rgb.blue) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.green)
        hsv.h = 85 + 43 * (rgb.blue - rgb.red) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.red - rgb.green) / (rgbMax - rgbMin);

    return hsv;
}

/**************************************************************************/
/*!
    @brief Converts HSV data into RGB data
    @param hsv struct. 
    @returns rgb struct. 
*/
/**************************************************************************/
rgb24 HsvToRgb(HsvColor hsv)
{
    rgb24 rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0){
        rgb.red = hsv.v;
        rgb.green = hsv.v;
        rgb.blue = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region){
    case 0:
        rgb.red = hsv.v; rgb.green = t; rgb.blue = p;
    break;
    case 1:
        rgb.red = q; rgb.green = hsv.v; rgb.blue = p;
    break;
    case 2:
        rgb.red = p; rgb.green = hsv.v; rgb.blue = t;
    break;
    case 3:
        rgb.red = p; rgb.green = q; rgb.blue = hsv.v;
    break;
    case 4:
        rgb.red = t; rgb.green = p; rgb.blue = hsv.v;
    break;
    default:
        rgb.red = hsv.v; rgb.green = p; rgb.blue = q;
    break;
    }

    return rgb;
}