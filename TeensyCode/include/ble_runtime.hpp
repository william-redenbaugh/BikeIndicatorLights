#ifndef _BLE_RUNTIME_HPP
#define _BLE_RUNTIME_HPP

#include "OS/OSThreadKernel.h"
#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"

// Protocall callback stuff. 
#include "MODULES/PROTOCALLBACKS/teensy_coms.h"

void setup_ble_runtime(void); 

#endif 