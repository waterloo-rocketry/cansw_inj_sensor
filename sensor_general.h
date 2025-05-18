#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#include <stdint.h>

#define LED_ON_R() (LATA4 = 0)
#define LED_OFF_R() (LATA4 = 1)


// Initialize LED
void LED_init(void);

void LED_heartbeat_R(void); // Red LED




