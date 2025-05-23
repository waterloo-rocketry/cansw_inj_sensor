#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#include <stdint.h>

// [TODO] pin assignment corresponds to KETO board for testing. 
// TO CHANGE PIN ASSIGNMENT LATER
#define LED_ON_R() (LATB3 = 0)
#define LED_OFF_R() (LATB3 = 1)
#define LED_ON_G() (LATB2 = 0)
#define LED_OFF_G() (LATB2 = 1)
#define LED_ON_Y() (LATB1 = 0)
#define LED_OFF_Y() (LATB1 = 1)


// Initialize LED
void LED_init(void);

void LED_heartbeat_R(void); // Red LED

#endif /* SENSOR_GEN_H */


