#ifndef _IMU_RUNTIME_HPP
#define _IMU_RUNTIME_HPP
#include <Arduino.h> 

#include "OS/OSThreadKernel.h"
#include "OS/OSSignalKernel.h"
#include "OS/OSMutexKernel.h"

#include "mpu6050_imu.h"

void setup_imu_runtime(void); 

#endif 