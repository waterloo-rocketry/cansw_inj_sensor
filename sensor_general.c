#include <stdbool.h>
#include <xc.h>

#include "mcc_generated_files/system/system.h"
#include <math.h>

#include "sensor_general.h"

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

    if (voltage_raw < 400) {
        voltage_raw = 400;
    } else if (voltage_raw > 2000) {
        voltage_raw = 2000;
    }

    float v = (voltage_raw + 0.5f) / 4096.0f * VREF;

    const uint16_t r = 100;
    const double pressure_range = 1450;

    double current = v / r;

    int32_t pressure_psi = (int32_t)(((current - 0.004) / (0.02 - 0.004)) * pressure_range);

    return (uint32_t)pressure_psi;
}

// Low-pass filter for 4-20mA pressure transducer
#define SAMPLE_FREQ(DIFF_ms) (1000.0 / DIFF_ms)
#define LOW_PASS_ALPHA(TR, DIFF_ms)                                                                \
    ((SAMPLE_FREQ(DIFF_ms) * TR / 5.0) / (1 + SAMPLE_FREQ(DIFF_ms) * TR / 5.0))
#define LOW_PASS_RESPONSE_TIME 2.5 // seconds
#define ALPHA_LOW(DIFF_ms) LOW_PASS_ALPHA(LOW_PASS_RESPONSE_TIME, DIFF_ms)

uint16_t update_pressure_psi_low_pass(
    adcc_channel_t adc_channel, double *low_pass_pressure_psi, uint16_t diff_ms
) {
    int16_t pressure_psi = get_pressure_4_20_psi(adc_channel);

    *low_pass_pressure_psi =
        ALPHA_LOW(diff_ms) * (*low_pass_pressure_psi) + (1.0 - ALPHA_LOW(diff_ms)) * pressure_psi;

    return (uint16_t)(*low_pass_pressure_psi);
}

uint16_t get_hall_sensor_reading(adcc_channel_t adc_channel) {
    return ADCC_GetSingleConversion(adc_channel);
}

