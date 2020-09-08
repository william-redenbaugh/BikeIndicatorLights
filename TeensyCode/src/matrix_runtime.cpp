#include "matrix_runtime.h"

/*!
*   @brief Statically allocated thread stack. 
*/
static uint32_t matrix_thread_stack[8192]; 

void start_matrix_runtime(void);
static void matrix_thread(void *parameters); 

void start_matrix_runtime(void){
    os_add_thread(&matrix_thread, NULL, sizeof(matrix_thread_stack), matrix_thread_stack); 
}

static void matrix_thread(void *parameters){
    setup_matrix_engine(255); 
    for(;;){
        os_thread_delay_ms(20);
    }
}