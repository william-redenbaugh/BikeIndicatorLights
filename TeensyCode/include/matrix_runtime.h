#ifndef _MATRIX_RUNTIME_H
#define _MATRIX_RUNTIME_H

#include <Arduino.h> 

#include "OS/OSMutexKernel.h"
#include "OS/OSSignalKernel.h"
#include "OS/OSThreadKernel.h"

#include "MatrixEngine.hpp"

void start_matrix_runtime(void); 

#endif