#include "battery_checking_runtime.hpp"

/*!
*   @brief Helper module that lets us read data from battery easily and intuitively. s
*/
static VoltageRead primary_battery; 

/*!
*   @brief Flag to determine if a battery is dead. 
*/
static OSSignal battery_dead;

/*!
*   @brief How full is the battery
*/
static float latest_percentage_read = 0.00; 

/*!
*   @brief The interval that our battery runtime thread goes at
*/
static const int BATTERY_RUNTIME_LPTHREAD_INTERVAL_MS = 10000; 

/*!
*   @brief Pin that we are using to read the battery voltage. 
*/
static const int BATTERY_GPIO = 12; 

/*!
*   @brief The dividor of the voltage reader(based off however you divide down the voltage to be read by gpio)
*/
static const int BATTERY_VOLTAGE_DIVIDER = 5; 

/*!
*   @brief Function declaration
*/
void init_battery_checking_runtime(void); 
static void battery_checking_periodic_func(void *parameters); 

/*!
*   @brief Initialization of module that checks the battery of the teensy module
*/
void init_battery_checking_runtime(void){
    primary_battery.init(BATTERY_GPIO, BATTERY_VOLTAGE_DIVIDER); 

    // Min and max ranges for 12v battery comprised of AA batteries. 
    primary_battery.configureBattery(11.2, 13.2); 

    // Check the battery module for anything every 1000 seconds. 
    add_lwip_task(&battery_checking_periodic_func, NULL, BATTERY_RUNTIME_LPTHREAD_INTERVAL_MS); 
}

static void battery_checking_periodic_func(void *parameters){
    latest_percentage_read = primary_battery.batteryPercentage(); 

    // Battery is essentially dead. 
    if(latest_percentage_read < .1)
        battery_dead.signal(THREAD_SIGNAL_0); 
    else
        battery_dead.clear(THREAD_SIGNAL_0); 
}