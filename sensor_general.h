#ifndef SENSOR_GEN_H
#define SENSOR_GEN_H

#include "mcc_generated_files/adc/adcc.h"
#include <stdint.h>

#define LED_R LATA4 // RA4
#define LED_W LATA3
#define LED_B LATA2
#define LED_OFF 1

// Initialize LEDS
void LED_init(void);

void LED_heartbeat_R(void); // Red LED

uint8_t board_error_checks();

// Read pressure sensor ADC and convert to PSI. Replace all negative values with
// zero since canlib and RLCS don't like it.
uint16_t update_pressure_psi_low_pass(adcc_channel_t adc_channel, double *low_pass_pressure_psi);
uint16_t get_hall_sensor_reading(adcc_channel_t adc_channel);
#endif /* SENSOR_GEN_H */

