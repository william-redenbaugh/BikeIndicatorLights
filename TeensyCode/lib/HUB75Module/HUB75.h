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

extern void setup_matrix(void); 

#endif 