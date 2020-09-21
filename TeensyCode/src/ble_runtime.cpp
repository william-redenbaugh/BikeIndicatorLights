#include "ble_runtime.hpp"

/*!
*   @brief Size of our ble thread runtime stack in uint32_t
*/
static const uint32_t BLE_RUNTIME_STACK_SIZE  = 2048; 

/*!
*   @brief BLE thread stack 
*/
static uint32_t ble_thread_stack[BLE_RUNTIME_STACK_SIZE]; 

/*!
*   @brief Function declarations
*/
void setup_ble_runtime(void); 
static void ble_thread(void *parameters); 

/*!
*   @brief Callback function declarations 
*/
static void signal_stop_callback(MessageReq *req); 
static void signal_stop_fast_callback(MessageReq *req); 
static void signal_white_callback(MessageReq *req); 
static void signal_turn_left_callback(MessageReq *req); 
static void signal_turn_right_callback(MessageReq *req); 
static void signal_turn_left_stop_callback(MessageReq *req); 
static void signal_turn_right_stop_callback(MessageReq *req); 

/*!
*   @brief Start up BLE thread, initialize ble subsystems. 
*/
void setup_ble_runtime(void){
    // Setting up our BLE logic runtime thread .
    os_add_thread(&ble_thread, NULL, BLE_RUNTIME_STACK_SIZE, ble_thread_stack); 
    // Setting up our message callbacks 
    message_callbacks_begin(&Serial1, 9600);
    
    // Adding our message callbacks for the different enumerated messages. 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_STOP, signal_stop_callback); 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_STOP_FAST, signal_stop_fast_callback); 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_WHITE, signal_white_callback); 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_TURN_LEFT, signal_turn_left_callback); 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_TURN_LEFT_STOP, signal_turn_left_stop_callback);
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_TURN_RIGHT, signal_turn_right_callback); 
    add_message_callback(MessageData_MessageType_BIKE_LED_SIGNAL_TURN_RIGHT_STOP, signal_turn_right_stop_callback); 
}

/*!
*   @brief ble logic runtime thread. 
*   @param void general purpose pointer 
*/
void ble_thread(void *parameters){
    for(;;){
        os_thread_delay_ms(10); 
    }
}

static void signal_stop_callback(MessageReq *req){
    
}

static void signal_stop_fast_callback(MessageReq *req){

}

static void signal_white_callback(MessageReq *req){

}

static void signal_turn_left_callback(MessageReq *req){

}

static void signal_turn_right_callback(MessageReq *req){

}

static void signal_turn_left_stop_callback(MessageReq *req){

}

static void signal_turn_right_stop_callback(MessageReq *req){

}