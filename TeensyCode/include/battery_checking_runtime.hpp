#ifndef _BATTERY_CHECKING_RUNTIME_HPP
#define _BATTERY_CHECKING_RUNTIME_HPP

// General arduino stuff. 
#include <Arduino.h>

// LPwork thread module that we throw the battery checking activities onto
#include "MODULES/LPWORK/lp_work_thread.h"

// So can check bits. 
#include "OS/OSSignalKernel.h"

// Battery Detect module
#include "HAL/VOLTAGE_READ/voltage_read.h"

void init_battery_checking_runtime(void);

#endif 