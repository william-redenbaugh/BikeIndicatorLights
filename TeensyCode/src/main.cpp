#include <Arduino.h> 
#include "OS/OSThreadKernel.h"
#include "MatrixEngine.hpp"

void setup() {
  os_init();   
  setup_matrix_engine(200);  
}

/*!
    @brief Fills a single frame for the matrix_cycle_individual function. Set's each RGB LED value.  
*/
__attribute__((always_inline)) static void matrix_cycle_individual_fillframe(uint8_t h){
    for(uint8_t y = 0; y < 4; y++){
        for(uint8_t x = 0; x < 4; x++){
          uint8_t hue_val = (uint8_t)(((y + 2) * (2 + x)* 12) + (h * 5)) % 255;
          HsvColor h = {hue_val, 255, 255};    
          set_matrix_hsv(x, y, h);
        }
    }
}

void loop() {
  // Looping runtime. 
  for(uint8_t h = 0; h < 255; h++){
      // Fills in each individual keyframe 
      matrix_cycle_individual_fillframe(h);  
      os_thread_delay_ms(15);
  }   
}