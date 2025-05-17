#include <stdbool.h>
#include <xc.h>
#include <math.h>

#include "sensor_general.h"

void LED_init(void) {
    TRISA2 = 0; // set A2 as output
    LATA2 = 1; // LED off by default
}


// Green LED
void LED_heartbeat_G(void) {
    static bool led_on = false;
    if (led_on) {
        LED_OFF_G();
        led_on = false;
    } else {
        LED_ON_G();
        led_on = true;
    }
}



