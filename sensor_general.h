#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#include <stdint.h>

#define LED_ON_G() (LATA2 = 0)
#define LED_OFF_G() (LATA2 = 1)


// Initialize LED
void LED_init(void);

void LED_heartbeat_G(void); // Green LED




