#include <xc.h>

#include "canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"

#include "timer.h"

// From 5V bus line. At this current, a warning will be sent out over CAN
#define BUS_OVERCURRENT_THRESHOLD_mA 100
#define BAT_OVERCURRENT_THRESHOLD_mA 150

const float VREF = 4.096;
const float mA_SENSE_CONVERT_FACTOR =
    10000 * VREF / 4096.0f; //  10e6 uV conversion / 100 V/V multiplier * FVR vref / 12bit adc
//******************************************************************************
//                              STATUS CHECKS                                 //
//******************************************************************************

// Check bus current error
bool check_5v_current_error(adcc_channel_t current_channel) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 62; // 62 is R8 rating in mR

    if (curr_draw_mA > BUS_OVERCURRENT_THRESHOLD_mA) {
        return false;
    }

    // things look ok
    return true;
}

// check battery current error
bool check_12v_current_error(adcc_channel_t current_channel) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(current_channel);
    float uV = voltage_raw * mA_SENSE_CONVERT_FACTOR;
    uint16_t curr_draw_mA = uV / 15; // 15 is R7 rating in mR

    if (curr_draw_mA > BAT_OVERCURRENT_THRESHOLD_mA) {
        return false;
    }

    // things look ok
    return true;
}

// checks if PT current is between 4 and 20 mA
bool check_PT_current_error(adcc_channel_t pt_channel) {
    adc_result_t voltage_raw = ADCC_GetSingleConversion(pt_channel);
    float v = (voltage_raw + 0.5f) / 4096.0f * VREF;
    const uint16_t r = 100;

    double current_mA = 1000 * v / r;

    if (current_mA < 4 || current_mA > 20) {
        return false;
    }

    // current is nominal
    return true;
}

