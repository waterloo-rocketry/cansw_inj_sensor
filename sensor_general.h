#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H


#include "mcc_generated_files/adc/adcc.h"
#include <stdint.h>

// [TODO] pin assignment corresponds to KETO board for testing. 
// TO CHANGE PIN ASSIGNMENT LATER
#define LED_ON_R() (LATB3 = 0) // RA4
#define LED_OFF_R() (LATB3 = 1)
#define LED_ON_Y() (LATB1 = 0) // RA3
#define LED_OFF_Y() (LATB1 = 1)
#define LED_ON_G() (LATB2 = 0) //RA2
#define LED_OFF_G() (LATB2 = 1)


// Initialize LEDS
void LED_init(void);

void LED_heartbeat_R(void); // Red LED

// Read pressure sensor ADC and convert to PSI. Replace all negative values with
// zero since canlib and RLCS don't like it.
uint16_t update_pressure_psi_low_pass(adcc_channel_t adc_channel, double *low_pass_pressure_psi);
uint16_t get_hall_sensor_reading(adcc_channel_t adc_channel);
#endif /* SENSOR_GEN_H */


