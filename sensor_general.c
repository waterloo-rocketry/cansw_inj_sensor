#include <stdbool.h>
#include <xc.h>
#include <math.h>

#include "sensor_general.h"

// [TODO] pin assignment corresponds to KETO board for testing. 
// TO CHANGE PIN ASSIGNMENT LATER
void LED_init(void) {
    TRISB3 = 0; // set B3 as output
    LATB3 = 1; // LED off by default
    TRISB2 = 0; // set B2 as output
    LATB2 = 1; // LED off by default
    TRISB1 = 0; // set B2 as output
    LATB1 = 1; // LED off by default
}


// Red LED
void LED_heartbeat_R(void) {
    static bool led_on = false;
    if (led_on) {
        LED_OFF_R();
        led_on = false;
    } else {
        LED_ON_R();
        led_on = true;
    }
}



