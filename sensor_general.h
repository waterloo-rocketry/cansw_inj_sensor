#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#include <stdint.h>

#define LED_ON_R() (LATB3 = 0)
#define LED_OFF_R() (LATB3 = 1)


// Initialize LED
void LED_init(void);

void LED_heartbeat_R(void); // Red LED

#endif /* SENSOR_GEN_H */


