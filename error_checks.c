#include <xc.h>

#include "canlib.h"

#include "mcc_generated_files/system/system.h"

#include "error_checks.h"

#include "timer.h"

#define OVER_VOLTAGE_12V_BITFIELD 0x04
#define UNDER_VOLTAGE_12V_BITFIELD 0x05

// [TODO] same board so assuming Vref is the same 
const float BATT_CONVERT_FACTOR = 1000 * 3.3 / 4096.0f; // mV conversion * vref / 12bit adc


bool check_battery_voltage_error(adcc_channel_t battery_channel, uint32_t *general_error_bitfield) { // returns mV
    // [TODO] does this do what i think it does, ie. get current input voltage?
    adc_result_t batt_raw = ADCC_GetSingleConversion(battery_channel);
    
    // [TODO] same board so assuming Vref is the same
    // Vref: 3.3V, Resolution: 12 bits -> raw ADC value is precisely in mV
    uint16_t batt_voltage_mV = (float)batt_raw * BATT_CONVERT_FACTOR;
    
    // [TODO] same board assuming correct
    // get the un-scaled battery voltage (voltage divider)
    // we don't care too much about precision - some truncation is fine
    batt_voltage_mV = batt_voltage_mV * 4;
    
    if (batt_voltage_mV < INPUT_UNDERVOLTAGE_THRESHOLD_mV ||
        batt_voltage_mV > INPUT_OVERVOLTAGE_THRESHOLD_mV) {

        uint32_t timestamp = millis();
        // [TODO] change to appropriate bitwise shift
        uint8_t volt_data[2] = {0};
        volt_data[0] = (batt_voltage_mV >> 8) & 0xff;
        volt_data[1] = (batt_voltage_mV >> 0) & 0xff;
//        enum BOARD_STATUS error_code = batt_voltage_mV < ACTUATOR_BATT_UNDERVOLTAGE_THRESHOLD_mV
//                                           ? E_BATT_UNDER_VOLTAGE
//                                           : E_BATT_OVER_VOLTAGE;

        can_msg_t error_msg;
        // [TODO] deprecated, need to find/build new P.S. check build_general_board_status_msg
        // assume general_board_status is passed as pointer. how does this file get access to that?? do i have to pass it in parent function?
//        build_board_stat_msg(timestamp, error_code, batt_data, 2, &error_msg);
//        txb_enqueue(&error_msg);

        return false;
    }
    
}




