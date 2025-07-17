#include <stdbool.h>
#include <xc.h>

#include "mcc_generated_files/system/system.h"
#include <math.h>

#include "sensor_general.h"

#define PRES_TIME_DIFF_ms 50 // 20 Hz
#define PT_OFFSET 0

const float VREF = 4.096; // FVR vref

void LED_init(void) {
    TRISA4 = 0;
    LED_R = !LED_OFF;
    TRISA3 = 0;
    LED_W = LED_OFF;
    TRISA2 = 0;
    LED_B = LED_OFF;
}


// 4-20mA pressure transducer
uint32_t get_pressure_4_20_psi(adcc_channel_t adc_channel) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(adc_channel);

    float v = (voltage_raw + 0.5f) / 4096.0f * VREF;

    const uint16_t r = 100;
    const double pressure_range = 1450;

    double current = v / r;

    int32_t pressure_psi = (int32_t)(((current - 0.004) / (0.02 - 0.004)) * pressure_range);

    return (uint32_t)pressure_psi + PT_OFFSET;
}

// Low-pass filter for 4-20mA pressure transducer
#define SAMPLE_FREQ (1000.0 / PRES_TIME_DIFF_ms)
#define LOW_PASS_ALPHA(TR) ((SAMPLE_FREQ * TR / 5.0) / (1 + SAMPLE_FREQ * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 2.5 // seconds
volatile double alpha_low = LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME);

uint16_t update_pressure_psi_low_pass(adcc_channel_t adc_channel, double *low_pass_pressure_psi) {
    int16_t pressure_psi = get_pressure_4_20_psi(adc_channel);

    *low_pass_pressure_psi =
        alpha_low * (*low_pass_pressure_psi) + (1.0 - alpha_low) * pressure_psi;

    return (uint16_t)(*low_pass_pressure_psi);
}

uint16_t get_hall_sensor_reading(adcc_channel_t adc_channel) {
    return ADCC_GetSingleConversion(adc_channel);
}

