#include <stdbool.h>
#include <xc.h>
#include <math.h>

#include "sensor_general.h"

void LED_init(void) {
    TRISB3 = 0; // set A4 as output
    LATB3 = 1; // LED off by default
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



